import asyncio
import websockets
import json
import logging
import time
import numpy as np
from reachy2_sdk import ReachySDK


class MujocoWSClient:
    def __init__(self, uri="ws://localhost:8765"):
        self.uri = uri
    async def send_request(self, payload):

        print(f"[MujocoWSClient] Connecting to {self.uri}...")
        

        async with websockets.connect(self.uri) as ws:
            print("[MujocoWSClient] Connected. Sending request.")
            
            try:
                await ws.send(json.dumps(payload))
                msg = await ws.recv()
                result = json.loads(msg)
                
                print("[MujocoWSClient] Received response. Disconnecting (via 'async with').")
                return result

            except Exception as e:
                print(f"[MujocoWSClient] Error during request: {e}")
                raise

    async def reset_env(self):
        """Réinitialise la simulation MuJoCo (async)."""
        payload = {"type": "reset"}
        return await self.send_request(payload)

    async def get_pose_in_frame(self, frame: str, target: str):
        payload = {"type": "get_pose_in_frame", "frame": frame, "target": target}
        return await self.send_request(payload)

class TaskServer:
    def __init__(self, reachy, num_episodes: int, reset_time_s: float):
        self.reachy = reachy
        self.num_episodes = num_episodes
        self.reset_time_s = reset_time_s

        self.events = {
            "exit_early": False,
            "rerecord_episode": False,
            "stop_recording": False,
            "ready_for_next_episode": False,
        }
        self.clients = set()
        self.event_listeners = set() 
        self.mujoco_ws=MujocoWSClient("ws://localhost:8765")
        # self.mujoco_ws.connect()

    async def broadcast_events(self):
        message = json.dumps({"events": self.events})
        listeners_to_remove = set()
        for ws in self.event_listeners:
            try:
                # Envoi asynchrone
                await ws.send(message)
            except websockets.exceptions.ConnectionClosed:
                listeners_to_remove.add(ws)
            except Exception as e:
                logging.error(f"[TaskServer] error when updating the states: {e}")
                listeners_to_remove.add(ws)
        

        self.event_listeners.difference_update(listeners_to_remove)

    async def handler(self, websocket):
        self.clients.add(websocket)
        logging.info("[TaskServer] Client connecté.")
        try:
            async for message in websocket:
                msg = json.loads(message)
                cmd = msg.get("cmd")

                if cmd == "register_listener": 
                    self.event_listeners.add(websocket)
                    logging.info("[TaskServer] Client enregistré comme auditeur.")
                    await self.broadcast_events()
                    

                elif cmd == "signal_ready":
                    self.events["ready_for_next_episode"] = True
                    await self.broadcast_events()
                    await websocket.send(json.dumps({"ack": "ok"}))

                elif cmd == "rerecord_episode":
                    self.events["rerecord_episode"] = True
                    await self.broadcast_events()
                    await websocket.send(json.dumps({"ack": "ok"}))
                
                elif cmd == "exit_early":
                    self.events["exit_early"] = False
                    await self.broadcast_events()
                    await websocket.send(json.dumps({"ack": "ok"}))

        except Exception as e:
            logging.error(f"[TaskServer] Erreur WebSocket: {e}")
        finally:
            self.clients.discard(websocket)
            self.event_listeners.discard(websocket)
            logging.info("[TaskServer] Client déconnecté.")

    
    async def reset(self):
        self.reachy.goto_posture()
        self.reachy.mobile_base.turn_off()
        self.reachy.mobile_base.turn_on()
        # await self.mujoco_ws.reset_env()   # ← libère la boucle ici
        await asyncio.sleep(2)

    def wait_for_gripper(self,arm):
        while arm.gripper.is_moving():
            time.sleep(0.1)
        time.sleep(0.5)

    def move_head(self,target_world_pose: np.ndarray):
        self.reachy.head.look_at(float(target_world_pose[0]), float(target_world_pose[1]), float(target_world_pose[2]))
    
    def get_to_waiting_pose(self, duration: float = 2.0) -> None:

        logging.info("[Task] Reachy start the task")

        self.reachy.r_arm.goto([30, 10, -15, -115, 0, 0, -15], duration)
        self.reachy.l_arm.goto([30, -10, 15, -115, 0, 0, 15], duration)
        
        self.reachy.r_arm.gripper.open()
        self.reachy.l_arm.gripper.open()
        
        self.wait_for_gripper(self.reachy.r_arm)
        self.wait_for_gripper(self.reachy.l_arm)

        time.sleep(duration)

        self.reachy.r_arm.translate_by(0.15,0.17,0, frame='robot',wait=True)
        self.reachy.r_arm.translate_by(0.09,0,-0.05, frame='robot',wait=True)

        self.reachy.r_arm.gripper.close()
        self.wait_for_gripper(self.reachy.r_arm)

        self.reachy.r_arm.translate_by(0,-0.18,0.08, frame='robot',wait=True)
        self.reachy.mobile_base.goto(x=0.2, y=0.0, theta=0.0,wait=True)
        self.reachy.r_arm.gripper.open()
        self.wait_for_gripper(self.reachy.r_arm)

        self.reachy.mobile_base.goto(x=-0.2, y=0.0, theta=0.0,wait=True)
        self.reachy.goto_posture()

        logging.info("[Task] Reachy finished the task")

       
    

    async def run_task_logic(self):
   
        await self.reset()
        await self.mujoco_ws.reset_env()
        await asyncio.sleep(self.reset_time_s)

        for episode in range(self.num_episodes):

            while not self.events["ready_for_next_episode"] :
                await asyncio.sleep(0.1)

            self.events["ready_for_next_episode"] = False
            await self.broadcast_events()

            logging.info(f"[TaskServer] Start the {episode+1}")

            try:
                self.get_to_waiting_pose()


                self.events["exit_early"] = True
                await self.broadcast_events()


                await asyncio.sleep(self.reset_time_s)
                await self.reset()
                await self.mujoco_ws.reset_env()
                self.events["exit_early"] = True
                await self.broadcast_events()

            except Exception as e:
                logging.error(f"[TaskServer] Erreur épisode {episode+1}: {e}")
                self.events["rerecord_episode"] = True
                continue

        self.events["stop_recording"] = True
        await self.broadcast_events()
        logging.info("[TaskServer] Toutes les tâches terminées.")

    async def start(self, host="0.0.0.0", port=6666):
        async with websockets.serve(self.handler, host, port):
            logging.info(f"[TaskServer] listen on ws://{host}:{port}")
            await self.run_task_logic()


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)


    reachy = ReachySDK(host="localhost")

    server = TaskServer(reachy, num_episodes=5, reset_time_s=3.0)

    asyncio.run(server.start())