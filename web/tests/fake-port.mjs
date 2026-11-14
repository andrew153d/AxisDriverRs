// A stand-in for a Web Serial port with a board on the other end.

import { decode, encode, frame, unframe } from "../protocol.js";

/**
 * A SerialPort whose device answers each request with what `answer` returns
 * for it: the fields of a Response other than its `id`, or raw bytes to
 * deliver, or a list of chunks of bytes to deliver one after another, or
 * nothing to stay silent. `unplug()` makes the port fail like a board that
 * was pulled out.
 */
export function fakePort(answer) {
  let incoming;
  return {
    isOpen: false,
    async open() {
      this.isOpen = true;
    },
    async close() {
      this.isOpen = false;
    },
    readable: new ReadableStream({
      start(controller) {
        incoming = controller;
      },
    }),
    writable: new WritableStream({
      async write(bytes) {
        const request = decode("Request", unframe(bytes).payload);
        let reply = answer(request);
        if (!reply) {
          return;
        }
        if (!(reply instanceof Uint8Array) && !Array.isArray(reply)) {
          reply = frame(encode("Response", { id: request.id, ...reply }));
        }
        for (const chunk of Array.isArray(reply) ? reply : [reply]) {
          incoming.enqueue(chunk);
          await new Promise((resolve) => setTimeout(resolve, 1));
        }
      },
    }),
    unplug() {
      incoming.error(new Error("The device has been lost."));
    },
  };
}
