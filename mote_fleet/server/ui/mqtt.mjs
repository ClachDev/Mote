// A subscribe-only MQTT 3.1.1 client over WebSockets.
//
// The fleet UI reads the control plane straight from the broker — no polling
// and no service in the middle (fleet.md Q5) — and a browser cannot speak raw
// MQTT, so the read path is MQTT-over-WebSockets. This is that client.
//
// It is subscribe-only in the strongest sense available to a client: there is
// no PUBLISH encoder in this file. The browser cannot publish a command even if
// something in the page tries, because the packet it would need does not exist
// here. Dispatch goes through the fleet API, which authorizes the operator and
// writes the audit line (fleet.md Q5/Q7); this half of the split is enforced by
// omission rather than by intention.
//
// Hand-rolled rather than a vendored MQTT library because what is needed is
// five packet types of a published wire format, and a minified blob in the tree
// is a dependency nobody can review. The packet codec is pure and exported, so
// `test/ui_test.mjs` runs it under node with no browser.

export const CONNECT = 1;
export const CONNACK = 2;
export const PUBLISH = 3;
export const PUBACK = 4;
export const SUBSCRIBE = 8;
export const SUBACK = 9;
export const PINGREQ = 12;
export const PINGRESP = 13;
export const DISCONNECT = 14;

const encoder = new TextEncoder();
const decoder = new TextDecoder();

// MQTT's variable-length integer: seven bits per byte, high bit continues.
export function encodeLength(value) {
  const bytes = [];
  let remaining = value;
  do {
    let byte = remaining % 128;
    remaining = Math.floor(remaining / 128);
    if (remaining > 0) byte |= 0x80;
    bytes.push(byte);
  } while (remaining > 0);
  return bytes;
}

function encodeString(text) {
  const bytes = encoder.encode(text);
  return [bytes.length >> 8, bytes.length & 0xff, ...bytes];
}

function packet(type, flags, body) {
  return new Uint8Array([(type << 4) | flags, ...encodeLength(body.length), ...body]);
}

// Connect flags (MQTT 3.1.1 §3.1.2.3). Only these three are ever set: there is
// no will to register, and this client has nothing to publish.
const CLEAN_SESSION = 0x02;
const HAS_PASSWORD = 0x40;
const HAS_USERNAME = 0x80;

export function encodeConnect(clientId, keepalive = 30, credential = null) {
  // Since M7 the broker is not anonymous, and what this carries is the
  // operator's own subscribe-only login, fetched from /v1/config with their
  // token. It grants the four read topics and no write anywhere — so "the
  // browser cannot publish" is now the broker's rule, not only this file's
  // omission of a PUBLISH encoder.
  let flags = CLEAN_SESSION;
  if (credential && credential.username) {
    flags |= HAS_USERNAME;
    if (credential.password) flags |= HAS_PASSWORD;
  }
  const body = [
    ...encodeString('MQTT'),
    4, // protocol level 3.1.1
    flags,
    keepalive >> 8,
    keepalive & 0xff,
    ...encodeString(clientId),
  ];
  if (flags & HAS_USERNAME) body.push(...encodeString(credential.username));
  if (flags & HAS_PASSWORD) body.push(...encodeString(credential.password));
  return packet(CONNECT, 0, body);
}

export function encodeSubscribe(packetId, filters, qos = 1) {
  const body = [packetId >> 8, packetId & 0xff];
  for (const filter of filters) body.push(...encodeString(filter), qos);
  return packet(SUBSCRIBE, 0x02, body); // 0x02 is mandatory for SUBSCRIBE
}

export function encodePuback(packetId) {
  return packet(PUBACK, 0, [packetId >> 8, packetId & 0xff]);
}

export function encodePingreq() {
  return packet(PINGREQ, 0, []);
}

export function encodeDisconnect() {
  return packet(DISCONNECT, 0, []);
}

// Parse whatever whole packets are in `buffer`, returning them plus the bytes
// left over: a WebSocket frame is not an MQTT packet boundary, so a packet can
// arrive split across frames or three at a time in one.
export function parsePackets(buffer) {
  const packets = [];
  let offset = 0;
  while (offset < buffer.length) {
    const start = offset;
    const type = buffer[offset] >> 4;
    const flags = buffer[offset] & 0x0f;
    offset += 1;
    let length = 0;
    let multiplier = 1;
    let byte;
    do {
      if (offset >= buffer.length) return { packets, rest: buffer.subarray(start) };
      byte = buffer[offset++];
      length += (byte & 0x7f) * multiplier;
      multiplier *= 128;
      if (multiplier > 128 ** 4) throw new Error('malformed remaining length');
    } while (byte & 0x80);
    if (offset + length > buffer.length) return { packets, rest: buffer.subarray(start) };
    packets.push(decodePacket(type, flags, buffer.subarray(offset, offset + length)));
    offset += length;
  }
  return { packets, rest: buffer.subarray(offset) };
}

function decodePacket(type, flags, body) {
  if (type === CONNACK) {
    return { type, sessionPresent: (body[0] & 1) === 1, returnCode: body[1] };
  }
  if (type === SUBACK) {
    return { type, packetId: (body[0] << 8) | body[1], granted: [...body.subarray(2)] };
  }
  if (type === PUBLISH) {
    const qos = (flags >> 1) & 0x03;
    const topicLength = (body[0] << 8) | body[1];
    let offset = 2 + topicLength;
    const topic = decoder.decode(body.subarray(2, offset));
    let packetId = null;
    if (qos > 0) {
      packetId = (body[offset] << 8) | body[offset + 1];
      offset += 2;
    }
    return {
      type,
      topic,
      qos,
      packetId,
      retain: (flags & 0x01) === 1,
      payload: decoder.decode(body.subarray(offset)),
    };
  }
  return { type, flags, body };
}

const CONNACK_REASONS = {
  1: 'the broker refused protocol version 3.1.1',
  2: 'the broker rejected the client id',
  3: 'the broker is unavailable',
  4: 'bad username or password',
  5: 'not authorized to connect',
};

// The client: connect, subscribe, hand every message to `onMessage`, and keep
// reconnecting. A fleet UI left open on a wall display has to survive the fleet
// box restarting without somebody pressing reload.
export class BrokerReader {
  constructor({ url, topics, onMessage, onState, clientId, keepalive = 30, credential = null }) {
    this.url = url;
    this.topics = topics;
    this.credential = credential;
    this.onMessage = onMessage || (() => {});
    this.onState = onState || (() => {});
    this.clientId = clientId || `mote-ui-${Math.random().toString(16).slice(2, 10)}`;
    this.keepalive = keepalive;
    this.socket = null;
    this.buffer = new Uint8Array(0);
    this.packetId = 0;
    this.pinger = null;
    this.retry = null;
    this.backoff = 1000;
    this.closed = false;
  }

  connect() {
    this.closed = false;
    this.onState('connecting', this.url);
    let socket;
    try {
      socket = new WebSocket(this.url, 'mqtt');
    } catch (error) {
      this._reconnect(String(error));
      return;
    }
    socket.binaryType = 'arraybuffer';
    this.socket = socket;
    socket.onopen = () =>
      socket.send(encodeConnect(this.clientId, this.keepalive, this.credential));
    socket.onmessage = (event) => this._onBytes(new Uint8Array(event.data));
    socket.onerror = () => {};
    socket.onclose = () => {
      this._stopPinging();
      if (!this.closed) this._reconnect('connection closed');
    };
  }

  close() {
    this.closed = true;
    this._stopPinging();
    if (this.retry) clearTimeout(this.retry);
    if (this.socket && this.socket.readyState === WebSocket.OPEN) {
      this.socket.send(encodeDisconnect());
      this.socket.close();
    }
    this.socket = null;
  }

  _reconnect(reason) {
    this.socket = null;
    this.buffer = new Uint8Array(0);
    this.onState('offline', reason);
    if (this.closed) return;
    this.retry = setTimeout(() => this.connect(), this.backoff);
    this.backoff = Math.min(this.backoff * 2, 30000);
  }

  _onBytes(chunk) {
    const merged = new Uint8Array(this.buffer.length + chunk.length);
    merged.set(this.buffer);
    merged.set(chunk, this.buffer.length);
    let parsed;
    try {
      parsed = parsePackets(merged);
    } catch (error) {
      this.onState('offline', String(error));
      if (this.socket) this.socket.close();
      return;
    }
    this.buffer = parsed.rest;
    for (const message of parsed.packets) this._onPacket(message);
  }

  _onPacket(message) {
    if (message.type === CONNACK) {
      if (message.returnCode !== 0) {
        this.onState(
          'refused',
          CONNACK_REASONS[message.returnCode] || `CONNACK ${message.returnCode}`,
        );
        if (this.socket) this.socket.close();
        return;
      }
      this.backoff = 1000;
      this.packetId = (this.packetId % 65535) + 1;
      this.socket.send(encodeSubscribe(this.packetId, this.topics));
      this._startPinging();
      this.onState('connected', this.url);
    } else if (message.type === PUBLISH) {
      // QoS 1 obliges an acknowledgement; without it the broker redelivers
      // every retained message on a timer and the roster flickers.
      if (message.qos === 1 && this.socket) {
        this.socket.send(encodePuback(message.packetId));
      }
      this.onMessage(message.topic, message.payload, message.retain);
    }
  }

  _startPinging() {
    this._stopPinging();
    this.pinger = setInterval(() => {
      if (this.socket && this.socket.readyState === WebSocket.OPEN) {
        this.socket.send(encodePingreq());
      }
    }, (this.keepalive * 1000) / 2);
  }

  _stopPinging() {
    if (this.pinger) clearInterval(this.pinger);
    this.pinger = null;
  }
}

// `mote/v1/<robot_id>/<leaf>` -> `{robotId, leaf}`; null for anything else.
export function parseTopic(topic, root = 'mote/v1') {
  const prefix = `${root}/`;
  if (!topic.startsWith(prefix)) return null;
  const rest = topic.slice(prefix.length);
  const slash = rest.indexOf('/');
  if (slash < 1) return null;
  return { robotId: rest.slice(0, slash), leaf: rest.slice(slash + 1) };
}
