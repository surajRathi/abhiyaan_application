#! /usr/bin/env python

# Higher value indicates higher precedence.
precedence = {
    '+': 2,
    '-': 2,
    '*': 5,
    '/': 5,
    '^': 10
}

# Maps operators to anonymous functions
binary_operators = {  # IMP: Stack pops second operand before first!!!!
    '+': lambda x, y: x + y,
    '-': lambda x, y: y - x,
    '*': lambda x, y: x * y,
    '/': lambda x, y: y / x,
    '^': lambda x, y: y ** x
}

openers = '([{'
for opener in openers:
    precedence[opener] = 0
closers = ')]}'

nums = '123456789.'

symbols = ''.join(binary_operators.keys()) + openers + closers
valid_characters = symbols + nums


# Could use a logic which alternates between reading symbols and numbers to verify the expression.
def tokenizer(expr: str, num_chars=nums):  # expr: Iterable[char] -> Generator[Union[chr, float], None, None]:
    """A generator which takes a string and breaks it down into tokens."""
    i = 0
    while i < len(expr):
        i_0 = i
        if expr[i] in num_chars:
            while (i < len(expr)) and (expr[i] in num_chars):
                i += 1
            yield float(expr[i_0:i])
        else:
            yield expr[i]
            i += 1


class InvalidExpressionError(Exception):
    """The passed arithmetic is not a valid expression."""
    pass


# Using https://stackoverflow.com/questions/13421424/how-to-evaluate-an-infix-expression-in-just-one-scan-using-stacks
def evaluate(expr: str) -> float:
    """Arithmetically evaluates an expression in the form of a string."""

    expr = ''.join(filter(lambda ch: ch in valid_characters, expr))
    if not expr:
        return float('NaN')  # raise error instead?

    operators = []
    operands = []

    try:
        for t in tokenizer(expr):
            if isinstance(t, float):
                operands.append(t)
            elif t in openers:
                operators.append(t)
            elif t in binary_operators:
                while operators and precedence[operators[-1]] >= precedence[t]:
                    operands.append(binary_operators[operators.pop()](operands.pop(), operands.pop()))
                operators.append(t)
            else:
                opener = openers[closers.index(t)]
                while (op := operators.pop()) != opener:
                    operands.append(binary_operators[op](operands.pop(), operands.pop()))

        while operators:
            operands.append(binary_operators[operators.pop()](operands.pop(), operands.pop()))

    except ArithmeticError as e:
        raise e
    except IndexError:
        raise InvalidExpressionError()

    #  assert (len(operands) == 1)
    return operands.pop()


def main():
    print('Arithmetic evaluator.')
    try:
        while True:
            print()
            print('> ', end='')
            try:
                print(evaluate(input()))
            except (ArithmeticError, InvalidExpressionError):
                print("Error.")
    except EOFError:
        pass

    print()


if __name__ == '__main__':
    main()
