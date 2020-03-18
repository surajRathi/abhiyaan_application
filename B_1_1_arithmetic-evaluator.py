#! /usr/bin/env python

# TODO: Support unary minus.
# Assumptions:
# The input has reasonable bounds on magnitude.
# No unary operators.

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
closers = ')]}'

for opener in openers:
    precedence[opener] = 0

nums = '0123456789.e'

symbols = ''.join(binary_operators.keys()) + openers + closers
valid_characters = symbols + nums

intro_string = """\
Arithmetic Evaluator.
Supports addition(+), subtraction(-), multiplication(*), division(/), and power(^) 
of floating point numbers. Precedence can be altered with '()', '[]', or '{}'.
Supports 'e' notation for numbers, and previous answer substitution with '!!'.
Does not support negative inputs.. Use EOF (^D) to quit."""


# Could use a logic which alternates between reading symbols and numbers to verify the expression.
def tokenizer(expr: str, num_chars=nums):  # expr: Iterable[char] -> Generator[Union[chr, float], None, None]:
    """A generator which takes a string and breaks it down into tokens."""
    i = 0
    while i < len(expr):
        if expr[i] in num_chars:
            i_0 = i
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

    # 'Stacks'
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
    except (ValueError, IndexError):  # One of the stacks runs out, i.e. invalid expression structure.
        raise InvalidExpressionError()

    #  assert (len(operands) == 1)
    return operands.pop()


def main():
    print(intro_string)

    ans = 0.0  # Used for previous answer substitution.

    try:
        while True:
            print()
            print('> ', end='')
            try:
                ans = evaluate(
                    input().replace('!!', str(ans), -1)
                )
                print(ans)
            except (ArithmeticError, InvalidExpressionError):
                print("Error.")
                ans = 0.0
    except EOFError:
        pass

    print()


if __name__ == '__main__':
    main()
