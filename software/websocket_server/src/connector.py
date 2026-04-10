import json
import time
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.websockets import WebSocketState
from constants import LOBBY_ID_LENGTH
from data.lobby import Lobby
from typing import Dict
from components.challenge import *
from utils.util import *
from logging import *

import traceback

class ServerWrapper:
    app: FastAPI
    lobbies: Dict[str, Lobby[WebSocket]]
    connections: Dict[WebSocket, str]
    configs: Dict
    log: Logger
    def __init__(self, cfg):
        self.app = FastAPI()

        self.app.add_api_websocket_route("/ws", endpoint=self.connectToLobby)
        self.lobbies = dict()
        self.connections = dict()
        self.configs = cfg
        self.log = getLogger("APP")
        

    def createLobby(self) -> Lobby:
        lobbyId = generate_str(LOBBY_ID_LENGTH)
        lobby = Lobby(lobbyId)
        self.lobbies[lobbyId] = lobby
        return lobby

    async def connectToLobby(self, webSocket: WebSocket):
        await webSocket.accept()

        nonce = make_nonce()
        ts = int(time.time())
        challenge = {
            "type": "challenge",
            "nonce": nonce,
            "timestamp": ts
        }

        await webSocket.send_json(challenge)

        try:
            data = await webSocket.receive_json()
            print("Data: ", data)
            if data.get("type") != "auth":
                print("Incorrect response type.")
                await webSocket.close(code=1008)
                return

            device_id = data.get("device_id")
            signature = data.get("signature")
            if not device_id or not signature:
                print("Incorrect data format.")
                await webSocket.close(code=1008)
                return
            
            secret = self.configs["data"].get(device_id)
            if not secret:
                print("Incorrect ID of the device.")
                await webSocket.close(code=1008)
                return
            now = int(time.time())
            if abs(now - ts) > self.configs["maxConnectionDifference"]:
                print("Await time is exceeded.")
                await webSocket.close(code=1008)
                return
            msg=f"{device_id}|{nonce}|{ts}"
            auth = sign(secret, msg)
            print(auth)
            if not compare_entries(auth, signature):
                print("Incorrect signature details.")
                await webSocket.close(code=1008)
                return
            await webSocket.send_json({"type": "auth_ok"})
            while webSocket.client_state == WebSocketState.CONNECTED:
                request = await webSocket.receive_json()
                await self.handle_request(webSocket, request)
        except WebSocketDisconnect:
            await self.clear_user(webSocket)
            self.log.info(f"Client: {webSocket.client.host} have been disconnected")
            return
        except Exception:
            await self.clear_user(webSocket)
            self.log.error(traceback.format_exc())
            await webSocket.close(code=1011)
            return
        
    async def handle_request(self, user: WebSocket, request: Dict):
        msg_type = request.get("type")
        if msg_type == "create_lobby":
            lobby = self.createLobby()
            lobby_id = lobby.get_lobby_id()
            await user.send_json(convert_message("lobby", lobby_id))
            lobby.add_client(user)
            self.connections[user] = lobby_id
            return
        if msg_type == "connect_lobby":
            id = request.get("id")
            if not id:
                await user.send_json(convert_message("err", "Id parameter is not provided!"))
                return
            lobby = self.lobbies.get(id)
            if not lobby:
                await user.send_json(convert_message("err", f"Lobby with id {id} not found!"))
                return
            lobby.add_client(user)
            self.connections[user] = id
            for lobbyUser in lobby.iterator():
                if lobbyUser != user:
                    await lobbyUser.send_json(convert_message("connection", user.client.host))
            return
        lobby_id = self.connections.get(user)
        if not lobby_id:
            await user.send_json(convert_message("err", "You are not connected to lobby!"))
            return
        lobby = self.lobbies.get(lobby_id)
        if not lobby:
            await user.send_json(convert_message("err", "Lobby not found!"))
            return
        for mate in lobby.iterator():
            await mate.send_json(request)
        return
            

    async def clear_user(self, user: WebSocket):
        lobby_id = self.connections.get(user)
        if not lobby_id:
            return
        lobby = self.lobbies.get(lobby_id)
        if lobby:
            lobby.remove_client(user)
            for connection in lobby.iterator():
                await connection.send_json(convert_message("disconnect", user.client.host))
        self.connections.pop(user, None)
        return
            

            


        
        


        
