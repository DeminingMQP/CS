#from graphics import *
import tester

def displayGrid(Grid):
    print(len(Grid))
    # win = GraphWin("Minefield", 600, 600)
    # graphicGrid = []
    # for i in range(0, len(Grid)):
    #     graphicGrid.append([])
    #     for j in range(0, len(Grid[0])):
    #         graphicGrid[i].append(
    #             Rectangle(Point(50 + 10 * i - 5, 500 - 10 * j - 5), Point(50 + 10 * i + 5, 500 - 10 * j + 5)))
    #         graphicGrid[i][j].draw(win)
    # for i in range(len(Grid[0]) - 1, -1, -1):
    #     for j in range(0, len(Grid)):
    #         if Grid[j][i] == 1:
    #             graphicGrid[j][i].setFill('black')
    #         if Grid[j][i] == 0:
    #             graphicGrid[j][i].setFill('white')

if __name__=="__main__":
    input("Press Enter to continue...")
    myGrid = tester.passGrid()
    print(myGrid)
    displayGrid(myGrid)