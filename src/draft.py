import shapely.affinity
from shapely.geometry import Point, Polygon

from matplotlib import pyplot as plt

from typing import List, TypedDict, Union

class A:
    def test(self):
        pass

class B:
    def test(self):
        pass

class C:
    pass

a = A()
c = C()

if isinstance(a, (A, B)):
    print("a is good")
else:
    print("a is bad")