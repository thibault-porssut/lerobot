import asyncio
import json
import websockets
import threading
import time
import copy
from typing import Optional, Any, Dict


def _async_send_command(uri: str, payload: Dict[str, Any]):
    async def send():
        try:
            async with websockets.connect(uri) as ws:
                await ws.send(json.dumps(payload))
          
                msg = await ws.recv() 
                return json.loads(msg)
        except Exception as e:
            return {"error": str(e)}
    return send()


class TaskClient:
    def __init__(self, uri="ws://localhost:6666"):
        self.uri = uri
        self._lock = threading.Lock()
        self._events: Dict[str, Any] = {
            "exit_early": False,
            "rerecord_episode": False,
            "stop_recording": False,
            "ready_for_next_episode": False,
        }
        self._is_running = False
        self._loop: Optional[asyncio.AbstractEventLoop] = None
        self._thread: Optional[threading.Thread] = None

    def _run_loop(self):
        self._loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self._loop)
        self._is_running = True
        self._loop.run_until_complete(self._connect_and_listen())
        self._is_running = False

    async def _connect_and_listen(self):
        print(f"[TaskClient] Start listenning on {self.uri}")
        while self._is_running:
            try:
                async with websockets.connect(self.uri) as ws:
                    print("[TaskClient] Connected. Record as listener...")
                    await ws.send(json.dumps({"cmd": "register_listener"})) 
                    
                    async for message in ws:
                        msg = json.loads(message)
                        if "events" in msg:
                           
                            with self._lock:
                                self._events.update(msg["events"])    

            except websockets.exceptions.ConnectionClosed:
                if self._is_running:
                    print("[TaskClient] Closed connexion. Reconnexion in 1s.")
                    await asyncio.sleep(1)
            except Exception as e:
                if self._is_running:
                    print(f"[TaskClient] Crtical Error: {e}. Reconnexion in 3s.")
                    await asyncio.sleep(3)
                else:
                    break
                
    def start(self):
        self._thread = threading.Thread(target=self._run_loop, daemon=True)
        self._thread.start()
        time.sleep(1) 

    def stop(self):
        self._is_running = False
        if self._loop:
            self._loop.call_soon_threadsafe(self._loop.stop)
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=5)

    
    def get_events(self):
        with self._lock:
            return copy.deepcopy(self._events)

    def _send_command_sync(self, cmd: str, payload: Optional[Dict[str, Any]] = None, timeout: float = 3.0):
        full_payload = {"cmd": cmd}
        if payload:
            full_payload.update(payload)
            
        coro = _async_send_command(self.uri, full_payload)
        
        try:
            return asyncio.run(coro)
        except Exception as e:
            print(f"[TaskClient] Échec de l'envoi de la commande {cmd}: {e}")
            return {"error": "command failed"}

    def signal_ready(self):
        return self._send_command_sync("signal_ready")

    def rerecord_episode(self):
        return self._send_command_sync("rerecord_episode")

    
    def exit(self):
        return self._send_command_sync("exit_early")
    
