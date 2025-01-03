class Adder():
    @staticmethod
    def add(a,b):
        return a + b

class Subtractor():
    @staticmethod
    def sub(a,b):
        return a - b
    
class NodeManager():
    _instance = None
    def __new__(cls):
        if cls._instance is None:
            cls._instance = super(NodeManager, cls).__new__(cls)
            cls._instance.adder = None
            cls._instance.subtractor = None
        return cls._instance
    
    @classmethod
    def get_adder(cls):
        if cls._instance is None:
            cls._instance = NodeManager()
        if cls._instance.adder is None:
            cls._instance.adder = Adder()
        return cls._instance.adder
    @classmethod
    def get_subtractor(cls):
        if cls._instance is None:
            cls._instance = NodeManager()
        if cls._instance.subtractor is None:
            cls._instance.subtractor = Subtractor()
        return cls._instance.subtractor
    
def main():
    adder = NodeManager.get_adder()
    print(adder.add(2, 3))
    subtractor = NodeManager.get_subtractor()
    print(subtractor.sub(3, 2))
    print(subtractor.sub(2,3))

if __name__ == '__main__':
    main()
