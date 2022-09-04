### pqdict ###
# doc: https://pqdict.readthedocs.io/en/latest/intro.html

# from pqdict import pqdict

# #default is a minheap
# pq = pqdict({'a':3, 'b':5, 'c':8})
# list(pq.popkeys())



# >>> pq.top()
# 'c'
# >>> pq.topitem()
# ('c', 1)
# # manual heapsort...
# >>> pq.pop()  # no args
# 'c'
# >>> pq.popitem()
# ('a', 3)
# >>> pq.popitem()
# ('b', 5)
# >>> pq.popitem()
# ('d', 6.5)
# >>> pq.popitem()  # ...and we're empty!
# KeyError

from pqdict import minpq
# pq = minpq(a=3, b=5, c=8)
# pq = minpq({'a':3, 'b':5, 'c':8})
pq = minpq()


pq["a"] = 16
print(pq.top())
# print(pq)