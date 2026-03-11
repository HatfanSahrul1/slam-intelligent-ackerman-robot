from abc import ABC, abstractmethod

class State(ABC):
    def __init__(self, node):
        self.node = node  # referensi ke node utama

    @abstractmethod
    def on_enter(self):
        """Dipanggil saat state dimasuki"""
        pass

    @abstractmethod
    def on_exit(self):
        """Dipanggil saat state ditinggalkan"""
        pass

    @abstractmethod
    def execute(self):
        """Logika utama state, dipanggil setiap iterasi"""
        pass

    @abstractmethod
    def next_state(self):
        """Tentukan state berikutnya (return string nama state atau None)"""
        return None