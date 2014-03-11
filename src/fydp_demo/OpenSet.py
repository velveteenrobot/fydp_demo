import heapq

class OpenSet:
    """Combination priority queue and dict for quick membership testing."""
    def __init__(self):
        self.heap = []
        self.dict = dict()
        self.desc = 0
        self.cleans = 0
        self.clean_thres = 0.10
        
    def push(self, priority, item):
        self.add(priority, item)
    
    def add(self, priority, item):
        heapq.heappush(self.heap, (priority, item))
        self.dict[(item[0], item[1])] = priority
        
    def peek(self):
        item = self.heap[0]
        running = True
        while running:
            if self.is_valid(item):
                running = False
            else:
                item = heapq.heappop(self.heap)
                self.desc = self.desc - 1
        return item    
        
    def pop(self):
        item = heapq.heappop(self.heap)
        running = True
        while running:
            if self.is_valid(item):
                running = False
            else:
                item = heapq.heappop(self.heap)
                self.desc = self.desc - 1
        del self.dict[(item[1][0], item[1][1])]   
        return item
    
    def contains(self, item):
        return (item[0], item[1]) in self.dict
        
    def is_valid(self, item):
        return self.contains(item[1]) and self.dict[(item[1][0], item[1][1])] == item[0]

    def remove(self, item):
        priority = self.dict[(item[0], item[1])]
        del self.dict[(item[0], item[1])]
        self.desc = self.desc + 1
        if len(self.dict) > 100 and self.desc / len(self.dict) > self.clean_thres:
            self.clean()
            
    def clean(self):
        l = zip( self.dict.values(), self.dict.keys())
        heapq.heapify(l)
        self.heap = l
        self.desc = 0
        self.cleans = self.cleans + 1       
        
    def  __len__(self):
        return len(self.dict)
