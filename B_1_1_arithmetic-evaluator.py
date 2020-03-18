#! /usr/bin/env python

"""unary_operators = {
    '+': lambda x: x,
    '-': lambda x: -x
}"""

binary_prec = {
    '+': 3,
    '-': 3,
    '*': 2,
    '/': 2,
    '^': 1
}

binary_operators = {
    '+': lambda x, y: x + y,
    '-': lambda x, y: x - y,
    '*': lambda x, y: x * y,
    '/': lambda x, y: x / y,
}

openers = '([{'
closers = ')]}'
symbols = ''.join(binary_operators.keys()) + openers + closers
nums = '123456789.'


def tokenizer(expr: str, num_chars=nums):  # expr: Iterable[char] -> Generator[Union[chr, float], None, None]:
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
    return  # Required?


def evaluate(expr: str) -> float:
    expr = ''.join(filter(lambda ch: ch in (symbols + nums), expr))
    if not expr:
        return float('NaN')  # raise error?

    # noinspection PyTypeChecker
    tks = tokenizer(expr)

    print(list(tks))
    return float('NaN')


def main():
    expr = input()

    print(evaluate(expr))


if __name__ == '__main__':
    main()
