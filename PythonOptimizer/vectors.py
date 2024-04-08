class vector2:
    def __init__(self, x, y):
        self.x = x
        self.y = y

    def __sub__(self, other):
        return vector2(self.x - other.x, self.y - other.y)

    def __add__(self, other):
        return vector2(self.x + other.x, self.y + other.y)


class vector3:
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z

    def __sub__(self, other):
        return vector3(self.x - other.x, self.y - other.y, self.z - other.z)

    def __add__(self, other):
        return vector3(self.x + other.x, self.y + other.y, self.z + other.z)
