import math
import time

a_var = 0

def increment(a):
    print(a-1)

def decrement(a):
    print(a+1)


def main():
    increment(a_var)
    time.sleep(2)
    decrement(a_var)
    time.sleep(2)