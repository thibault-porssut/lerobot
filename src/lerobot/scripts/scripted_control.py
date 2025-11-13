# Fichier: scripted_control.py
import logging
import threading

class ScriptedEventListener:
    """
    Remplace `init_keyboard_listener`. Contient le dictionnaire `events`
    et fournit des méthodes pour que `task.py` puisse le contrôler.
    """
    def __init__(self):
        self.events = {
            "exit_early": False,
            "rerecord_episode": False,
            "stop_recording": False,
        }
        logging.info("ScriptedEventListener initialisé.")
    
        self.ready_for_next_episode = threading.Event() 
        self.ready_for_next_episode.set() # Initialisé à True (prêt pour le premier épisode)
        logging.info("ScriptedEventListener initialisé.")

    def stop_episode(self):
        """Appelé par task.py pour terminer l'épisode en cours."""
        logging.info("[Event] Fin de l'épisode demandée par le script.")
        self.events["exit_early"] = True

    def rerecord_episode(self):
        """Appelé par task.py pour recommencer l'épisode."""
        logging.info("[Event] Ré-enregistrement de l'épisode demandé par le script.")
        self.events["rerecord_episode"] = True
        self.events["exit_early"] = True

    def stop_recording(self):
        """Appelé par task.py à la toute fin."""
        logging.info("[Event] Fin de l'enregistrement demandée par le script.")
        self.events["stop_recording"] = True
        self.events["exit_early"] = True
        
    def reset_episode_flags(self):
        """Appelé par le script d'enregistrement avant de commencer un nouvel épisode."""
        self.events["exit_early"] = False
        self.events["rerecord_episode"] = False
    
    def wait_for_ready(self):
        """Bloque le thread de tâche jusqu'à ce que le thread principal signale qu'il est prêt."""
        logging.info("[Event] Thread de tâche bloqué, en attente du thread principal...")
        self.ready_for_next_episode.wait() # Attend que l'événement soit "set"
        self.ready_for_next_episode.clear() # Le met à False pour la prochaine attente
        
    def signal_ready(self):
        """Signale au thread de tâche que le thread principal a terminé la sauvegarde et est prêt."""
        logging.info("[Event] Thread principal signale qu'il est prêt.")
        self.ready_for_next_episode.set()