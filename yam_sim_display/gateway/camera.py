"""One passthrough webrtcbin pipeline per synthetic RTP camera.

Media path (no decoder, encoder or converter)::

    udpsrc(HOST:PORT) ! rtpjitterbuffer ! rtph264depay ! h264parse
      ! rtph264pay ! queue ! webrtcbin -> DTLS/SRTP -> browser

GStreamer is imported here only, so configuration and signaling tests remain
pure stdlib.  The GLib main loop is owned by ``gateway/run.py``.
"""

from __future__ import annotations

import os
import re
import threading
import time

import gi

gi.require_version("Gst", "1.0")
gi.require_version("GstSdp", "1.0")
gi.require_version("GstWebRTC", "1.0")
from gi.repository import Gst, GstSdp, GstWebRTC  # noqa: E402

Gst.init(None)

RTP_CAPS = (
    "application/x-rtp,media=video,encoding-name=H264,"
    "payload=96,clock-rate=90000"
)
H264_CAPS = "application/x-rtp,media=video,encoding-name=H264"
ICE_GATHER_TIMEOUT_S = 5.0


def parse_sdp(text: str):
    """Convert an SDP string into the object GstWebRTC expects."""
    result, message = GstSdp.SDPMessage.new()
    if result != GstSdp.SDPResult.OK:
        raise RuntimeError(f"could not allocate SDP message: {result}")
    result = GstSdp.sdp_message_parse_buffer(text.encode("utf-8"), message)
    if result != GstSdp.SDPResult.OK:
        raise RuntimeError(f"could not parse SDP: {result}")
    return message


def struct_to_dict(value) -> dict:
    """Best-effort conversion of a Gst.Structure to plain Python."""
    out: dict = {}
    if not isinstance(value, Gst.Structure):
        return out
    for index in range(value.n_fields()):
        name = value.nth_field_name(index)
        try:
            item = value.get_value(name)
        except Exception:  # noqa: BLE001 - Gst values can be non-introspectable
            continue
        if isinstance(item, Gst.Structure):
            out[name] = struct_to_dict(item)
        elif isinstance(item, (int, float, str, bool)):
            out[name] = item
        else:
            out[name] = str(item)
    return out


def dump_elements(pipeline, path: str) -> list[str]:
    """Write every element factory in the pipeline, recursively."""
    names: list[str] = []
    iterator = pipeline.iterate_recurse()
    while True:
        try:
            result, element = iterator.next()
        except TypeError:
            item = iterator.next()
            if not item:
                break
            result, element = item
        if result != Gst.IteratorResult.OK or element is None:
            break
        factory = element.get_factory()
        names.append(factory.get_name() if factory else element.get_name())

    names = sorted(set(names))
    directory = os.path.dirname(path)
    if directory:
        os.makedirs(directory, exist_ok=True)
    with open(path, "w", encoding="utf-8") as handle:
        for name in names:
            handle.write(name + "\n")
    return names


class CameraPipeline:
    """A single-use sendonly H.264 WebRTC pipeline."""

    _sequence = 0

    def __init__(
        self, camera_id: str, host: str, port: int, run_dir: str
    ) -> None:
        CameraPipeline._sequence += 1
        self.sequence = CameraPipeline._sequence
        self.camera_id = camera_id
        self.host = host
        self.port = port
        self.run_dir = run_dir

        self.started = False
        self.answer_posted = False
        self.error: str | None = None
        self.ice_candidates: list[str] = []
        self.negotiated_pt: int | None = None
        self.connection_state = "new"
        self.ice_connection_state = "new"
        self.ice_gathering_state = "new"

        self._stats_lock = threading.Lock()
        self._ingress_packets = 0
        self._ingress_bytes = 0
        self._sink_pad = None
        self._bus = None
        self._elements_path = os.path.join(
            run_dir, f"gateway-elements-{camera_id}.txt"
        )

        suffix = f"{camera_id}_{self.sequence}"
        self.pipeline = Gst.Pipeline.new(f"yam_gateway_{suffix}")
        self.udpsrc = self._make("udpsrc", f"udpsrc_{suffix}")
        self.udpsrc.set_property("address", host)
        self.udpsrc.set_property("port", port)
        self.udpsrc.set_property("reuse", True)
        self.udpsrc.set_property("caps", Gst.Caps.from_string(RTP_CAPS))

        self.jitter = self._make("rtpjitterbuffer", f"jitter_{suffix}")
        self.jitter.set_property("latency", 50)
        self.jitter.set_property("drop-on-latency", True)

        self.depay = self._make("rtph264depay", f"depay_{suffix}")
        self.parse = self._make("h264parse", f"parse_{suffix}")
        self.parse.set_property("config-interval", -1)
        self.parse.set_property("disable-passthrough", True)

        self.pay = self._make("rtph264pay", f"pay_{suffix}")
        self.pay.set_property("pt", 96)
        self.pay.set_property("config-interval", -1)
        self.pay.set_property("mtu", 1200)

        self.queue = self._make("queue", f"queue_{suffix}")
        self.queue.set_property("max-size-buffers", 300)
        self.queue.set_property("max-size-bytes", 3_000_000)
        self.queue.set_property("max-size-time", 0)

        self.webrtc = self._make("webrtcbin", f"webrtc_{suffix}")
        self.webrtc.set_property("bundle-policy", "max-bundle")
        self.webrtc.set_property("latency", 0)

        for element in (
            self.udpsrc,
            self.jitter,
            self.depay,
            self.parse,
            self.pay,
            self.queue,
            self.webrtc,
        ):
            self.pipeline.add(element)

        self._link(self.udpsrc, self.jitter)
        self._link(self.jitter, self.depay)
        self._link(self.depay, self.parse)
        self._link(self.parse, self.pay)
        self._link(self.pay, self.queue)

        self.udpsrc.get_static_pad("src").add_probe(
            Gst.PadProbeType.BUFFER, self._on_rtp_buffer, None
        )
        self.webrtc.connect("on-ice-candidate", self._on_ice_candidate)
        self.webrtc.connect("notify::connection-state", self._on_state_notify)
        self.webrtc.connect("notify::ice-connection-state", self._on_state_notify)
        self.webrtc.connect("notify::ice-gathering-state", self._on_state_notify)

        # Answerer setup is done before the offer arrives.  A pad-created
        # transceiver otherwise has an unknown media kind and produces an
        # inactive answer.
        if self.pipeline.set_state(Gst.State.READY) == Gst.StateChangeReturn.FAILURE:
            raise RuntimeError(f"{camera_id}: pipeline could not reach READY")
        _state_return, state, _pending = self.pipeline.get_state(5 * Gst.SECOND)
        if state == Gst.State.NULL:
            raise RuntimeError(f"{camera_id}: pipeline stayed NULL")

        pad = self.webrtc.request_pad_simple("sink_%u")
        if pad is None:
            raise RuntimeError(f"{camera_id}: webrtcbin refused a sink pad")
        self._sink_pad = pad
        transceiver = pad.get_property("transceiver")
        transceiver.set_property("codec-preferences", Gst.Caps.from_string(H264_CAPS))
        transceiver.set_property(
            "direction", GstWebRTC.WebRTCRTPTransceiverDirection.SENDONLY
        )
        if self.queue.get_static_pad("src").link(pad) != Gst.PadLinkReturn.OK:
            raise RuntimeError(f"{camera_id}: failed to link payloader to webrtcbin")

        self._bus = self.pipeline.get_bus()
        self._bus.add_signal_watch()
        self._bus.connect("message::error", self._on_bus_error)

        self.initial_elements = dump_elements(self.pipeline, self._elements_path)
        print(
            f"[gateway] {camera_id}: RTP {host}:{port} -> rtpjitterbuffer -> "
            "rtph264depay -> h264parse -> rtph264pay -> webrtcbin "
            f"({len(self.initial_elements)} elements)",
            flush=True,
        )

    # -- construction helpers --------------------------------------------

    def _make(self, factory: str, name: str):
        element = Gst.ElementFactory.make(factory, name)
        if element is None:
            raise RuntimeError(f"{self.camera_id}: missing GStreamer element {factory}")
        return element

    @staticmethod
    def _link(src, dst) -> None:
        if not src.link(dst):
            raise RuntimeError(f"failed to link {src.name} -> {dst.name}")

    # -- callbacks --------------------------------------------------------

    def _on_rtp_buffer(self, _pad, info, _data):
        buffer = info.get_buffer()
        if buffer is not None:
            with self._stats_lock:
                self._ingress_packets += 1
                self._ingress_bytes += buffer.get_size()
        return Gst.PadProbeReturn.OK

    def _on_ice_candidate(self, _webrtc, _mline, candidate: str) -> None:
        self.ice_candidates.append(str(candidate))

    def _on_state_notify(self, webrtc, _property) -> None:
        self.connection_state = webrtc.get_property("connection-state").value_nick
        self.ice_connection_state = webrtc.get_property(
            "ice-connection-state"
        ).value_nick
        self.ice_gathering_state = webrtc.get_property(
            "ice-gathering-state"
        ).value_nick

    def _on_bus_error(self, _bus, message) -> None:
        error, debug = message.parse_error()
        self.error = f"{error.message} ({debug})"
        print(f"[gateway] {self.camera_id}: GStreamer error: {self.error}", flush=True)

    # -- negotiation ------------------------------------------------------

    def handle_offer(self, sdp: str) -> str:
        """Consume one browser offer and return a non-trickle answer."""
        if self.started:
            raise RuntimeError(f"{self.camera_id}: pipeline already negotiated")
        self.started = True

        offer_path = os.path.join(
            self.run_dir, f"gateway-{self.camera_id}-offer.sdp"
        )
        with open(offer_path, "w", encoding="utf-8") as handle:
            handle.write(sdp)

        description = GstWebRTC.WebRTCSessionDescription.new(
            GstWebRTC.WebRTCSDPType.OFFER, parse_sdp(sdp)
        )
        promise = Gst.Promise.new()
        self.webrtc.emit("set-remote-description", description, promise)
        promise.wait()
        _raise_if_rejected(promise, "set-remote-description")

        promise = Gst.Promise.new()
        self.webrtc.emit("create-answer", None, promise)
        promise.wait()
        _raise_if_rejected(promise, "create-answer")
        reply = promise.get_reply()
        if reply is None or not reply.has_field("answer"):
            raise RuntimeError(f"{self.camera_id}: create-answer returned no answer")
        answer = reply.get_value("answer")
        answer_sdp = answer.sdp.as_text()

        # The browser's answer fixes the H.264 payload type.  Re-stamp the
        # payloader so emitted RTP matches it instead of assuming 96.
        match = re.search(r"a=rtpmap:(\d+)\s+H264/90000", answer_sdp, re.IGNORECASE)
        if match:
            self.negotiated_pt = int(match.group(1))
            self.pay.set_property("pt", self.negotiated_pt)

        promise = Gst.Promise.new()
        self.webrtc.emit("set-local-description", answer, promise)
        promise.wait()
        _raise_if_rejected(promise, "set-local-description")

        self.pipeline.set_state(Gst.State.PLAYING)

        deadline = time.monotonic() + ICE_GATHER_TIMEOUT_S
        while time.monotonic() < deadline:
            if (
                self.webrtc.get_property("ice-gathering-state")
                == GstWebRTC.WebRTCICEGatheringState.COMPLETE
            ):
                break
            time.sleep(0.05)

        dump_elements(self.pipeline, self._elements_path)

        result = answer_sdp.rstrip()
        for candidate in self.ice_candidates:
            if candidate not in result:
                result += f"\r\na={candidate}"
        result += "\r\n"
        with open(
            os.path.join(self.run_dir, f"gateway-{self.camera_id}-answer.sdp"),
            "w",
            encoding="utf-8",
        ) as handle:
            handle.write(result)
        return result

    # -- stats / lifecycle ------------------------------------------------

    def status(self) -> dict:
        return {
            "connection_state": self.connection_state,
            "ice_connection_state": self.ice_connection_state,
            "ice_gathering_state": self.ice_gathering_state,
            "negotiated_pt": self.negotiated_pt,
            "ice_candidates": len(self.ice_candidates),
            "error": self.error,
            "answer_posted": self.answer_posted,
        }

    def stats(self) -> dict:
        outbound: dict = {}
        if self._sink_pad is not None:
            try:
                promise = Gst.Promise.new()
                self.webrtc.emit("get-stats", self._sink_pad, promise)
                promise.wait()
                reply = promise.get_reply()
                if reply is not None:
                    outbound = struct_to_dict(reply)
            except Exception as exc:  # noqa: BLE001 - stats must never kill media
                outbound = {"error": str(exc)}

        with self._stats_lock:
            packets = self._ingress_packets
            nbytes = self._ingress_bytes

        result = {
            "source": "gateway",
            "camera": self.camera_id,
            "host": self.host,
            "port": self.port,
            "ts": time.time(),
            "ingress_packets": packets,
            "ingress_bytes": nbytes,
            "webrtc": outbound,
        }
        result.update(self.status())
        return result

    def dispose(self) -> None:
        """Tear down this pipeline without affecting any other camera."""
        if self._bus is not None:
            try:
                self._bus.remove_signal_watch()
            except Exception:  # noqa: BLE001 - best effort during shutdown
                pass
            self._bus = None
        try:
            self.pipeline.set_state(Gst.State.NULL)
        except Exception:  # noqa: BLE001 - best effort during shutdown
            pass


def _raise_if_rejected(promise, action: str) -> None:
    reply = promise.get_reply()
    if reply is None:
        return
    if reply.has_field("error"):
        error = reply.get_value("error")
        raise RuntimeError(f"{action} failed: {error}")
