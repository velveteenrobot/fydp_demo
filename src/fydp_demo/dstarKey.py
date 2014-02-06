class dstarKey:
    def __init__(self, _k1, _k2):
        self.k1 = _k1
        self.k2 = _k2
        
    def __lt__(self, other):
        if self.k1 + 0.000001 < other.k1:
            return True
        elif self.k1 - 0.000001 > other.k1:
            return False
        return self.k2 < other.k2
    
    def __le__(self, other):
        if self.k1 < other.k1:
            return True
        elif self.k1 > other.k1:
            return False
        return self.k2 < other.k2 + 0.00001
        
    def __str__(self):
        return str(self.k1) + ", " + str(self.k2)
    
    def __repr__(self):
        return str(self.k1) + ", " + str(self.k2)