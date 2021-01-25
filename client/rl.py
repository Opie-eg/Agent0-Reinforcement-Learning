import client
import ast
import random
import time

VISITED_COLOR = "#400000"
FRONTIER_COLOR = "red3"


# AUXILIAR

class Queue:
    def __init__(self):
        self.queue_data = []

    def isEmpty(self):
        if len(self.queue_data) == 0:
            return True
        else:
            return False

    def pop(self):
        return self.queue_data.pop(0)

    def insert(self, element):
        return self.queue_data.append(element)

    def getQueue(self):
        return self.queue_data


# SEARCH AGENT

class Node:
    def __init__(self, state, parent, action, path_cost):
        self.state = state
        self.parent = parent
        self.action = action
        self.path_cost = path_cost

    def getState(self):
        return self.state

    def getParent(self):
        return self.parent

    def getAction(self):
        return self.action

    def getPathCost(self):
        return self.path_cost


class Agent:
    def __init__(self):
        self.c = client.Client('127.0.0.1', 50001)
        self.res = self.c.connect()
        random.seed()  # To become true random, a different seed is used! (clock time)
        self.visited_nodes = Queue()
        self.frontier_nodes = Queue()
        self.weightMap = []
        self.goalNodePos = (0, 0)
        self.state = (0, 0)
        self.maxCoord = (0, 0)

    def getConnection(self):
        return self.res

    def getGoalPosition(self):
        msg = self.c.execute("info", "goal")
        goal = ast.literal_eval(msg)
        # test
        print('Goal is located at:', goal)
        return goal

    def getSelfPosition(self):
        msg = self.c.execute("info", "position")
        pos = ast.literal_eval(msg)
        # test
        # print('Received agent\'s position:', pos)
        return pos

    def getRewards(self):
        msg = self.c.execute("info", "rewards")
        rewards = ast.literal_eval(msg)
        # test
        return rewards

    def getPatchCost(self, pos):
        return self.weightMap[pos[0]][pos[1]]

    def getMaxCoord(self):
        msg = self.c.execute("info", "maxcoord")
        max_coord = ast.literal_eval(msg)
        # test
        print('Received maxcoord', max_coord)
        return max_coord

    def getObstacles(self):
        msg = self.c.execute("info", "obstacles")
        obst = ast.literal_eval(msg)
        # test
        print('Received map of obstacles:', obst)
        return obst

    # COM MODIFICAÇÕES NO SERVIDOR
    def getObjectsAt(self, x, y):
        msg = self.c.execute("info", str(x) + "," + str(y))
        return ast.literal_eval(msg)

    # COM MODIFICAÇÕES NO SERVIDOR
    def isVisitable(self, x, y):
        return all(obj != "obstacle" and obj != "bomb" for obj in self.getObjectsAt(x, y))

    def step(self, pos, action):
        if action == "east":
            if pos[0] + 1 < self.maxCoord[0]:
                new_pos = (pos[0] + 1, pos[1])
            else:
                new_pos = (0, pos[1])

        if action == "west":
            if pos[0] - 1 >= 0:
                new_pos = (pos[0] - 1, pos[1])
            else:
                new_pos = (self.maxCoord[0] - 1, pos[1])

        if action == "north":
            if pos[1] + 1 < self.maxCoord[1]:
                new_pos = (pos[0], pos[1] + 1)
            else:
                new_pos = (pos[0], 0)

        if action == "south":
            if pos[1] - 1 >= 0:
                new_pos = (pos[0], pos[1] - 1)
            else:
                new_pos = (pos[0], self.maxCoord[1] - 1)
        return new_pos

    def getNode(self, parent_node, action):
        state = self.step(parent_node.getState(), action)
        # pathCost = parent_node.getPathCost() + self.getPatchCost(state)
        # return Node(state, parent_node, action, pathCost)
        return Node(state, parent_node, action, 0)

    def printNodes(self, type, nodes, i):
        print(type, " (round ", i, " )")
        print("state | path cost")
        for node in nodes.getQueue():
            print(node.getState(), "|", node.getPathCost())

    def printPath(self, node):
        n = node
        n_list = []
        while n.getPathCost() != 0:
            n_list.insert(0, [n.getState(), n.getPathCost()])
            n = n.getParent()
        print("Final Path", n_list)

    def mark_visited(self, node):
        # self.c.execute("mark_visited", str(node.getState())[1:-1].replace(" ", ""))
        self.c.execute("mark", str(node.getState())[1:-1].replace(" ", "") + "_" + VISITED_COLOR)

    def mark_frontier(self, node):
        # self.c.execute("mark_frontier", str(node.getState())[1:-1].replace(" ", ""))
        self.c.execute("mark", str(node.getState())[1:-1].replace(" ", "") + "_" + FRONTIER_COLOR)

    # Reinforce learning de aprendizagem
    def reinforcement_learning(self, episodios, qTable, rewards):
        # inicializar caminho a vazio, max_coord para sabermos dimensão do labirinto
        caminho = []
        max_coord = self.getMaxCoord()
        # inicializar qTable e rewards se os parâmetros forem nulos(1 vez a correr função)
        if qTable is None:
            qTable = []
            for i in range(0, max_coord[0]):
                qTable.append([])
                for j in range(0, max_coord[1]):
                    qTable[i].append([0.0, 0.0, 0.0, 0.0]) # cada "quadrado" do labirinto tem 4 valores para as
                                                           #   possíveis direções(norte; este; sul; oeste)
        if rewards is None:
            rewards = self.getRewards()

        goal_pos = self.getGoalPosition()
        posicao_atual = self.getSelfPosition()
        caminho.append(posicao_atual)
        c = self.c
        if c != -1:
            end = False
            msg = c.execute("command", "set_steps")
            # através de movimentos aleatórios tentar chegar ao goal
            while not end:
                msg = c.execute("info", "view")
                res = random.randint(0, 4)
                posicao_antiga = posicao_atual
                if res == 1:
                    c.execute("command", "right")
                if res == 2:
                    c.execute("command", "right")
                    c.execute("command", "right")
                else:
                    c.execute("command", "forward")
                    posicao_atual = self.getSelfPosition()
                    if posicao_atual != posicao_antiga:
                        caminho.append(posicao_atual)
                # verificar se o agente na mesma posição que o objetivo
                if posicao_atual == goal_pos:
                    end = True
                    print("Found the goal")

            # reverse the path and create matrix with values of rewards
            print("Caminho: ", caminho)
            caminho_revertido = caminho[::-1]
            goal = caminho_revertido[0]
            del caminho_revertido[0]

            #rewards[goal[0]][goal[1]] = 100 #Caso de nao importar os rewards, colocar pelo menos o do goal a 100
            valor_desconto = 0.8
            peso = 0
            mover = ""
            # inicializa-se ao goal para a primeira iteração do seguinte "for"
            ultima_posicao = goal

            for posicao in caminho_revertido:
                if posicao[0] == ultima_posicao[0]:  # X igual da última posicão e da nova
                    if posicao[1] - ultima_posicao[1] == 1: # caso a ultima posicao estiver mesmo acima da nova
                        mover = "north"
                    elif posicao[1] - ultima_posicao[1] == -1: # caso a ultima posicao estiver mesmo abaixo da nova
                        mover = "south"
                    # estes 2 abaixo tratam dos casos de "overflow" quando a ultima posicao se encontra num dos limites
                    #   inferior ou superior e a posicao no oposto
                    elif posicao[1] == 0 and ultima_posicao[1] == max_coord[1] - 1:
                        mover = "north"
                    elif posicao[1] == max_coord[1] - 1 and ultima_posicao[1] == 0:
                        mover = "south"

                elif posicao[1] == ultima_posicao[1]:  # Y igual da última posicão e da nova
                    if posicao[0] - ultima_posicao[0] == 1: # caso a ultima posicao estiver mesmo à esquerda da nova
                        mover = "west"
                    elif posicao[0] - ultima_posicao[0] == -1: # caso a ultima posicao estiver mesmo à direita da nova
                        mover = "east"
                    # estes 2 abaixo tratam dos casos de "overflow" quando a ultima posicao se encontra num dos limites
                    #   esquerdo ou direito e a posicao no oposto
                    elif posicao[0] == 0 and ultima_posicao[0] == max_coord[0] - 1:
                        mover = "west"
                    elif posicao[0] == max_coord[0] - 1 and ultima_posicao[0] == 0:
                        mover = "east"

                # reward da ultima posicao vai ser buscado à lista de rewards
                reward = rewards[ultima_posicao[0]][ultima_posicao[1]]
                # linha com valores da posicao atual é selecionado
                valorqTable = qTable[posicao[0]][posicao[1]]
                valor_q = peso + int(round((reward * valor_desconto), 0))
                # guardamos sempre apenas se for maior do que o valor guardado, visto que não queremos esquecer
                #    informação mais importante
                if valor_q > rewards[posicao[0]][posicao[1]]:
                    rewards[posicao[0]][posicao[1]] = valor_q

                # dependendo da direção atualiza-se o valor da qTable
                # Posicao 0->norte
                # Posicao 1->este
                # Posicao 2->sul
                # Posicao 3->oeste
                if mover == "north" and valor_q > valorqTable[0]:
                        qTable[posicao[0]][posicao[1]] = [valor_q, valorqTable[1], valorqTable[2], valorqTable[3]]

                elif mover == "east" and valor_q > valorqTable[1]:
                        qTable[posicao[0]][posicao[1]] = [valorqTable[0], valor_q, valorqTable[2], valorqTable[3]]

                elif mover == "south" and valor_q > valorqTable[2]:
                        qTable[posicao[0]][posicao[1]] = [valorqTable[0], valorqTable[1], valor_q, valorqTable[3]]

                elif mover == "west" and valor_q > valorqTable[3]:
                    qTable[posicao[0]][posicao[1]] = [valorqTable[0], valorqTable[1], valorqTable[2], valor_q]

                else:
                    qTable[posicao[0]][posicao[1]] = [valorqTable[0], valorqTable[1], valorqTable[2], valorqTable[3]]

                # atualiza-se ultima posicao para continuar a comparar(recursividade)
                ultima_posicao = posicao

            if episodios > 1:
                # colocamos agente de volta ao início
                c.execute("command", "home")
                # guardamos informacoes das tabelas qTable e dos rewards e diminuimos os episodios restantes
                self.reinforcement_learning(episodios - 1, qTable, rewards)
            else:
                # apresentar valores finais optidos
                print("qTable final", qTable)
                countY = -1
                # este ciclo for corre cada linha da qTable(cada possivel posicao) e escolhe o maior valor,

                for entry in qTable:
                    countX = -1
                    countY += 1
                    for i in entry:
                        mark = ""
                        countX += 1
                        countPos = -1
                        maior = -999
                        for j in i:
                            if j > maior:
                                countPos += 1
                                maior = j
                                #  atribuindo o valor(mark) à posicao, sendo = 0 norte = 1 este, =2 sul e =3 oeste
                                #  (ordem atribuida em cima)
                                if countPos == 0:
                                    mark = "north"
                                elif countPos == 1:
                                    mark = "east"
                                elif countPos == 2:
                                    mark = "south"
                                elif countPos == 3:
                                    mark = "west"
                            else:
                                countPos += 1
                        if mark != "":
                                c.execute("marrow", str(mark) + "," + str(countX) + "," + str(countY))
                        # para visualizar a marcação das setas 1 a 1
                        time.sleep(0.1)
                # para visualizar bem o resultado final
                time.sleep(10)
def main():
    print("Starting client!")
    ag = Agent()
    if ag.getConnection() != -1:
        # inciamos aprendizagem com 50 episódios
        ag.reinforcement_learning(50, None, None)


main()
