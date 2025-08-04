class Rule:
    def __init__(self, name, elements):
        self.ruleName = name
        self.members = elements
    
    def numSymbols(self) -> int:
        return len(self.members)
    
    def getElement(self,i) -> str:
        return self.members[i]