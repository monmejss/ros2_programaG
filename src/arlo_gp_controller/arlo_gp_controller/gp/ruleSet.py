from arlo_gp_controller.gp.rule import Rule

class RuleSet:
    def __init__(self,symbol):
        self.symbol = symbol
        self.Terminals = []
        self.NonTerminals = []

    def addTerminalRule(self, rule):
        self.Terminals.append(rule)

    def addNonTerminalRule(self, rule):
        self.NonTerminals.append(rule)
    
    def numTerminals(self) -> int:
        return len(self.Terminals)
    
    def numNonTerminals(self) -> int:
        return len(self.NonTerminals)

    def onlyTerminals(self) -> bool:
        return (self.numTerminals()>0 and self.numNonTerminals()==0)

    def onlyNonTerminals(self) -> bool:
        return (self.numNonTerminals()>0 and self.numTerminals()==0)

    def getRuleset(self):
        lst = []
        for rule in self.Terminals:
            lst.append(rule.ruleName)
        for rule in self.NonTerminals:
            lst.append(rule.ruleName)
        return lst
    def getTerminalSet(self):
        lst = []
        for rule in self.Terminals:
            lst.append(rule.ruleName)
        return lst
        
    def getNonTerminalSet(self):
        lst = []
        for rule in self.NonTerminals:
            lst.append(rule.ruleName)
        return lst

    