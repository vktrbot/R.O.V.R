import time
from typing import Set, TypeVar, Generic

T = TypeVar('T')

class Lobby(Generic[T]):
    __clients: Set[T]
    __createDate: float
    __lastActivityTime: float
    __lobbyId: str

    def __init__(self, lobbyId):
        self.__clients = set()
        self.__lobbyId = lobbyId
        self.__createDate = time.time()
        self.__lastActivityTime = time.time()

    def add_client(self, ws: T):
        self.__clients.add(ws)
        self.update_activity_time()
    
    def remove_client(self, ws: T):
        self.__clients.discard(ws)
        self.update_activity_time()

    def update_activity_time(self):
        self.__lastActivityTime = time.time()
    
    def get_inactivity_time(self):
        return time.time() - self.__lastActivityTime
    
    def get_lobby_id(self):
        return self.__lobbyId
    
    def iterator(self):
        for client in self.__clients:
            yield client
    
