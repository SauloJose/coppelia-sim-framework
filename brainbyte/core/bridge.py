import zmq
import numpy as np
import cbor2 

class SimulationBridge:
    def __init__(self, host="127.0.0.1", port=23001, timeout=10000):
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.REQ)
        self.socket.setsockopt(zmq.RCVTIMEO, timeout) 
        self.socket.connect(f"tcp://{host}:{port}")
        
        self.command_buffer = {'velocities': {}, 'positions': {}}
        self.latest_state = {}

    def queue_velocity(self, path: str, velocity: float):
        """ Agendar comandos de velocidade para juntas de robôs"""
        self.command_buffer['velocities'][path] = velocity

    def queue_position(self, path: str, position: float):
        """ Para braços robóticos ou servos """
        self.command_buffer['positions'][path] = position

    def get_sensor_data(self, path: str):
        """ Classes podem chamar para puxar o cachê, ou buffer de um determinado sensor"""
        return self.latest_state.get(path)

    def initialize(self, monitor_paths: list, actuator_paths: list, simulation) -> dict:
        payload = {"type": "INIT", "monitor": monitor_paths, "actuators": actuator_paths}
        
        self.socket.send(cbor2.dumps(payload))
        
        try:
            raw_data = self.socket.recv()
            return cbor2.loads(raw_data)
        except zmq.error.Again:
            raise ConnectionError("Timeout: O CoppeliaSim não respondeu ao INIT. A simulação está rodando?")
        
    def step(self):
        payload = self.command_buffer.copy()
        payload["type"] = "STEP"
        
        # Envia comandos
        self.socket.send(cbor2.dumps(payload))
        self.command_buffer = {'velocities': {}, 'positions': {}}
        
        # Recebe a resposta 
        raw_bytes = self.socket.recv()

        raw_state = cbor2.loads(raw_bytes)

        self.latest_state = {}
        for key, value in raw_state.items():
            # Processamento de dados binários do Coppelia (ex: LIDAR ou Visão)
            if key.endswith("_bin"):
                # O Coppelia usa float32 para o sim.packFloatTable
                self.latest_state[key] = np.frombuffer(value, dtype=np.float32)
            else:
                self.latest_state[key] = value
                
        return self.latest_state

    
    def queue_command(self, category: str, path: str, value):
        """ 
        Enfileira qualquer tipo de comando.
        Ex categorias: 'velocities', 'positions', 'teleports'
        """
        if category not in self.command_buffer:
            self.command_buffer[category] = {}
            
        self.command_buffer[category][path] = value
        
    def close(self):
        # Envia um comando de encerramento
        try:
            self.socket.setsockopt(zmq.RCVTIMEO, 100)
            self.socket.send(cbor2.dumps({"type": "CLOSE"}))
            self.socket.recv()
        except:
            pass
        finally:
            self.socket.close()
            self.context.term()