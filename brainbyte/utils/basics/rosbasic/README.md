# ROS Basics Templates

Esta pasta contém templates e guias para criar projetos ROS rapidamente no BRAINBYTE.

## 📁 Conteúdo

### Templates de Código (`.txt`)

1. **node_template.txt** - Template básico de um nó ROS
   - Estrutura de inicialização
   - Loop principal com `rospy.spin()`
   - Método de shutdown

2. **publisher_template.txt** - Template para publicar mensagens
   - Criação de publisher
   - Método de publicação
   - Loop de envio

3. **subscriber_template.txt** - Template para receber mensagens
   - Criação de subscriber
   - Callback de recebimento
   - Tratamento de mensagens

4. **service_template.txt** - Template para servidor de serviço
   - Criação de serviço
   - Handler de requisição
   - Loop do servidor

5. **service_client_template.txt** - Template para cliente de serviço
   - Criação de cliente
   - Chamada de serviço
   - Tratamento de resposta

### Guias de Referência (`.txt`)

6. **topics.txt** - Tudo sobre tópicos ROS
   - O que é um tópico
   - Nomenclatura recomendada
   - Tipos de mensagens comuns
   - Estrutura hierárquica

7. **nodes.txt** - Tudo sobre nós ROS
   - O que é um nó
   - Ciclo de vida
   - Tipos de nós
   - Boas práticas

8. **messages.txt** - Tudo sobre mensagens
   - Mensagens personalizadas
   - Tipos de dados
   - Como criar e compilar
   - Exemplos práticos

9. **parameters.txt** - Tudo sobre parametrização
   - O que é um parâmetro
   - Acesso em Python
   - Arquivos YAML
   - Launch files

10. **structure.txt** - Estrutura de projetos ROS
    - Organização de diretórios
    - Arquivos package.xml e CMakeLists.txt
    - Como compilar
    - Como executar

## 🚀 Como Usar

### Criar um Novo Projeto ROS

1. No BRAINBYTE, ative "Usar ROS Projects" em Configurações
2. Crie um novo projeto
3. Consulte o guia apropriado:
   - Leia `structure.txt` para organização geral
   - Leia `nodes.txt` para planejar seus nós
   - Leia `topics.txt` para definir comunicação
   
4. Use os templates para implementar:
   ```bash
   # Exemplo: copiar template de publisher
   cp brainbyte/utils/basics/rosbasic/publisher_template.txt meu_projeto/src/meu_package/nodes/sensor_pub.py
   ```

5. Substitua as placeholders:
   - `{name_node}` → nome do seu nó
   - `{topic_name}` → nome do tópico
   - etc.

### Desenvolvimento Típico

```
1. Estruturar o projeto (structure.txt)
   ↓
2. Planejar nós e tópicos (nodes.txt, topics.txt)
   ↓
3. Definir mensagens (messages.txt)
   ↓
4. Configurar parâmetros (parameters.txt)
   ↓
5. Implementar usando templates
   ↓
6. Testar e debug
```

## 📖 Exemplo Prático

### Criar um Publisher de Sensor

**Passo 1:** Copiar template
```bash
cat node_template.txt > sensor_publisher.py
```

**Passo 2:** Editar com substitutos
- `{name_node}` → `SensorPublisher`
- Adicionar lógica de publicação

**Passo 3:** Consultar guia
- Ler `topics.txt` para nomenclatura
- Ler `messages.txt` para tipo de mensagem
- Ler `parameters.txt` para configuração

**Passo 4:** Resultado final
```python
#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32

class SensorPublisher:
    def __init__(self):
        self.pub = rospy.Publisher('/robot/sensor/temperature', Float32, queue_size=10)
        rospy.init_node('sensor_publisher')
        self.rate = rospy.Rate(10)  # 10 Hz
    
    def run(self):
        while not rospy.is_shutdown():
            msg = Float32(data=25.5)  # Temperatura em Celsius
            self.pub.publish(msg)
            self.rate.sleep()

if __name__ == '__main__':
    node = SensorPublisher()
    node.run()
```

## 🎯 Estrutura Recomendada para Projetos ROS

```
meu_projeto/
├── src/
│   └── meu_package/
│       ├── package.xml
│       ├── CMakeLists.txt
│       ├── nodes/
│       │   ├── sensor_pub.py
│       │   └── motor_controller.py
│       ├── launch/
│       │   └── main.launch
│       ├── config/
│       │   └── params.yaml
│       └── msg/
│           └── SensorReading.msg
├── config/
│   └── description.txt (descrição do projeto)
└── README.md
```

## 📝 Checklist para Novo Projeto

- [ ] Criar estrutura de diretórios (ver structure.txt)
- [ ] Criar package.xml
- [ ] Criar CMakeLists.txt
- [ ] Definir nós no documento de planejamento
- [ ] Definir tópicos e tipos de mensagens
- [ ] Implementar cada nó usando templates
- [ ] Criar arquivo launch
- [ ] Configurar parâmetros em YAML
- [ ] Testar comunicação entre nós
- [ ] Documentar o projeto
- [ ] Adicionar arquivo README.md

## 🔗 Referências

Para mais informações sobre ROS:
- [ROS Documentation](http://wiki.ros.org/)
- [ROS Tutorials](http://wiki.ros.org/ROS/Tutorials)
- [Python ROS](http://wiki.ros.org/rospy)

## 💡 Dicas

- Sempre consulte os guias antes de começar
- Use templates como base - não reescreva do zero
- Teste cada componente isoladamente
- Use `rostopic echo` para debug
- Use `rosgraph` para visualizar a arquitetura
- Organize código com namespaces

---

**Última atualização:** 27/05/2026  
**Parte do projeto:** BRAINBYTE Robotics Framework
