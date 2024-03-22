#!/usr/bin/env python

#
# map.py
#

## TODO: https://realpython.com/documenting-python-code/
class Map(object):
    """docstring for Map."""

    def __init__(self):
        # True -> explored, obstacle
        # False -> explored, nothing found
        # None -> unexplored
        self.map = [
            [None, None, None, None, None],
            [None, '■', 'u', '■', None],
            [None, '■', '+', '■', None],
            [None, '■', '■', '■', None],
            [None, None, None, None, None]
        ]
        self.coor_or = ['up', 'right', 'down', 'left']
        self.coor = (2,2,'up')

    def __str__(self):
        map_str = ""
        for row in self.map:
            #map_str += " ".join(str(cell) if cell is not None else "░" for cell in row) + "\n"
            map_str += " ".join("·" if cell is False else
                                "x" if cell is True else
                                "░" if cell is None else
                                str(cell) for cell in row) + "\n"
        return map_str + str(self.coor)

    def _add_row_up(self):
        self.map.insert(0, [None] * len(self.map[0]))
        self.coor = (self.coor[0], self.coor[1] + 1, self.coor[2])

    def _add_row_down(self):
        self.map.append([None] * len(self.map[0]))

    def _add_column_left(self):
        for row in self.map:
            row.insert(0, None)
        self.coor = (self.coor[0] + 1, self.coor[1], self.coor[2])

    def _add_column_right(self):
        for row in self.map:
            row.append(None)

    def _checkandexpand(self):
        coor_x, coor_y, coor_or = self.coor
        # Aqui añadir + cols or rows
        if coor_or == 'up':
            if coor_y - 2 == 0:
                self._add_row_up()
        elif coor_or == 'down':
            if coor_y + 2 == len(self.map) - 1:
                self._add_row_down()
        elif coor_or == 'right':
            if coor_x + 2 == len(self.map[0]) - 1:
                self._add_column_right()
        else: # coor_or == 'left':
            if coor_x - 2 == 0:
                self._add_column_left()

    def moveFoward(self):
        self._checkandexpand()
        coor_x, coor_y, coor_or = self.coor

        if coor_or == 'up':
            self.map[coor_y - 2][coor_x-1:coor_x+2] = self.map[coor_y - 1][coor_x-1:coor_x+2]
            self.map[coor_y - 1][coor_x-1:coor_x+2] = self.map[coor_y][coor_x-1:coor_x+2]
            self.map[coor_y][coor_x-1:coor_x+2] = self.map[coor_y + 1][coor_x-1:coor_x+2]
            self.map[coor_y + 1][coor_x-1:coor_x+2] = [False, False, False]
            self.coor = (coor_x, coor_y - 1, coor_or)
        elif coor_or == 'down':
            self.map[coor_y + 2][coor_x-1:coor_x+2] = self.map[coor_y + 1][coor_x-1:coor_x+2]
            self.map[coor_y + 1][coor_x-1:coor_x+2] = self.map[coor_y][coor_x-1:coor_x+2]
            self.map[coor_y][coor_x-1:coor_x+2] = self.map[coor_y - 1][coor_x-1:coor_x+2]
            self.map[coor_y - 1][coor_x-1:coor_x+2] = [False, False, False]
            self.coor = (coor_x, coor_y + 1, coor_or)
        elif coor_or == 'right':
            self.map[coor_y - 1][coor_x:coor_x+3] = self.map[coor_y - 1][coor_x-1:coor_x+2]
            self.map[coor_y][coor_x:coor_x+3] = self.map[coor_y][coor_x-1:coor_x+2]
            self.map[coor_y + 1][coor_x:coor_x+3] = self.map[coor_y + 1][coor_x-1:coor_x+2]
            self.map[coor_y - 1][coor_x-1] = False
            self.map[coor_y][coor_x-1] = False
            self.map[coor_y + 1][coor_x-1] = False
            self.coor = (coor_x + 1, coor_y, coor_or)
        else: # coor_or == 'left':
            self.map[coor_y - 1][coor_x-2:coor_x+1] = self.map[coor_y - 1][coor_x-1:coor_x+2]
            self.map[coor_y][coor_x-2:coor_x+1] = self.map[coor_y][coor_x-1:coor_x+2]
            self.map[coor_y + 1][coor_x-2:coor_x+1] = self.map[coor_y + 1][coor_x-1:coor_x+2]
            self.map[coor_y - 1][coor_x+1] = False
            self.map[coor_y][coor_x+1] = False
            self.map[coor_y + 1][coor_x+1] = False
            self.coor = (coor_x - 1, coor_y, coor_or)

    def rotate(self, clockwise=True):
        coor_x, coor_y, coor_or = self.coor
        if clockwise:
            l1 = lambda x : x + 1
            l2 = lambda x : x - 1
        else:
            l1 = lambda x : x - 1
            l2 = lambda x : x + 1

        if coor_or == 'up':
            self.map[coor_y - 1][coor_x] = '■'
            self.map[coor_y][l1(coor_x)] = 'u'
        elif coor_or == 'right':
            self.map[coor_y][coor_x + 1] = '■'
            self.map[l1(coor_y)][coor_x] = 'u'
        elif coor_or == 'down':
            self.map[coor_y + 1][coor_x] = '■'
            self.map[coor_y][l2(coor_x)] = 'u'
        else: # coor_or == 'left':
            self.map[coor_y][coor_x - 1] = '■'
            self.map[l2(coor_y)][coor_x] = 'u'

        self.coor = (coor_x, coor_y, self.coor_or[l1(self.coor_or.index(coor_or)) & 3])

    def getinfo(self):
        coor_x, coor_y, coor_or = self.coor
        if coor_or == 'up':
            aux = self.map[coor_y - 2][coor_x-1:coor_x+2]
        elif coor_or == 'down':
            aux = self.map[coor_y + 2][coor_x-1:coor_x+2]
            aux.reverse()
        elif coor_or == 'right':
            aux = [self.map[coor_y - 1][coor_x + 2], self.map[coor_y][coor_x + 2],
                self.map[coor_y + 1][coor_x + 2]]
        else: # coor_or == 'left':
            aux = [self.map[coor_y - 1][coor_x - 2], self.map[coor_y][coor_x - 2],
                self.map[coor_y + 1][coor_x - 2]]
        return [False if x is None else x for x in aux]

    def drawObstacle(self, where = [False, False, False]):
        coor_x, coor_y, coor_or = self.coor
        if coor_or == 'up':
            self.map[coor_y - 2][coor_x-1:coor_x+2] = where
            if coor_y - 2 == 0:
                self._add_row_up()
        elif coor_or == 'down':
            where.reverse()
            self.map[coor_y + 2][coor_x-1:coor_x+2] = where
            if coor_y + 2 == len(self.map) - 1:
                self._add_row_down()
        elif coor_or == 'right':
            self.map[coor_y - 1][coor_x + 2] = where[0]
            self.map[coor_y][coor_x + 2] = where[1]
            self.map[coor_y + 1][coor_x + 2] = where[2]
            if coor_x + 2 == len(self.map[0]) - 1:
                self._add_column_right()
        else: # coor_or == 'left':
            self.map[coor_y - 1][coor_x - 2] = where[2]
            self.map[coor_y][coor_x - 2] = where[1]
            self.map[coor_y + 1][coor_x - 2] = where[0]
            if coor_x - 2 == 0:
                self._add_column_left()

    def drawObstacle_R(self, r = None):
        coor_x, coor_y, coor_or = self.coor
        if r is None:
            return
        if coor_or == 'up':
            self.map[coor_y - 1][coor_x + 2] = r or bool(self.map[coor_y - 1][coor_x + 2])
        elif coor_or == 'down':
            self.map[coor_y + 1][coor_x - 2] = r or bool(self.map[coor_y + 1][coor_x - 2])
        elif coor_or == 'right':
            self.map[coor_y + 2][coor_x + 1] = r or bool(self.map[coor_y + 2][coor_x + 1])
        else: # coor_or == 'left':
            self.map[coor_y - 2][coor_x - 1] = r or bool(self.map[coor_y - 2][coor_x - 1])

    def drawObstacle_L(self, l = None):
        coor_x, coor_y, coor_or = self.coor
        if l is None:
            return
        def _helper(a,b):
            a = bool(a) or b
        if coor_or == 'up':
            self.map[coor_y - 1][coor_x - 2] = l or bool(self.map[coor_y - 1][coor_x - 2])
        elif coor_or == 'down':
            self.map[coor_y + 1][coor_x + 2] = l or bool(self.map[coor_y + 1][coor_x + 2])
        elif coor_or == 'right':
            self.map[coor_y - 2][coor_x + 1] = l or bool(self.map[coor_y - 2][coor_x + 1])
        else: # coor_or == 'left':
            self.map[coor_y + 2][coor_x - 1] = l or bool(self.map[coor_y + 2][coor_x - 1])

    def test(self):
        print(str(self))
        print('move 4 + r')
        self.move()
        self.move()
        self.move()
        self.move()
        self.rotate()
        print(str(self))
        print('move 3 + r')
        self.move()
        self.move()
        self.move()
        self.rotate()
        print(str(self))
        print('move 4 + r')
        self.move()
        self.move()
        self.move()
        self.move()
        self.rotate()
        print(str(self))
        print('move 3 + r')
        self.move()
        self.move()
        self.move()
        self.rotate()
        print(str(self))
