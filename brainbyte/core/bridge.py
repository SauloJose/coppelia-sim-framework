import zmq
import numpy as np
import cbor2 

class SimulationBridge:
    def __init__(self, host="127.0.0.1", port=23001, timeout=10000):
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.REQ)
        self.socket.setsockopt(zmq.RCVTIMEO, timeout) 
        self.socket.connect(f"tcp://{host}:{port}")
        
        # O nosso "carrinho de compras" de comandos
        self.command_buffer = {'velocities': {}, 'positions': {}}
        self.latest_state = {}

    def queue_velocity(self, path: str, velocity: float):
        """ Classes como o TurtleBot chamam isso para agendar movimento """
        self.command_buffer['velocities'][path] = velocity

    def queue_position(self, path: str, position: float):
        """ Para braços robóticos ou servos """
        self.command_buffer['positions'][path] = position

    def get_sensor_data(self, path: str):
        """ Classes como o LDS_02 chamam isso para ler o cache local """
        return self.latest_state.get(path)

    def initialize(self, monitor_paths: list, actuator_paths: list, sim_handle) -> dict:
        payload = {"type": "INIT", "monitor": monitor_paths, "actuators": actuator_paths}
        
        self.socket.send(cbor2.dumps(payload))
        if sim_handle:
            sim_handle.step()

        raw_data = self.socket.recv()
        return cbor2.loads(raw_data)

    def step(self):
        payload = self.command_buffer.copy()
        payload["type"] = "STEP"
        
        # 1. Envia comandos
        self.socket.send(cbor2.dumps(payload))
        self.command_buffer = {'velocities': {}, 'positions': {}}
        
        # 2. Recebe a resposta (EM BYTES, não dê decode aqui!)
        raw_bytes = self.socket.recv()

       # 3. CBOR lê os bytes tranquilamente, incluindo as tabelas binárias do LIDAR
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
        self.socket.close()
        self.context.term()