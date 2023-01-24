from dataclasses import dataclass
from typing import Callable, Generator, Generic, Optional, Reversible, TypeVar

__all__ = ['LinkedList']


T = TypeVar("T")


class Node(Generic[T]):
    def __init__(self, value: T, next: Optional["Node[T]"] = None) -> None:
        super().__init__()
        self.__value = value
        # public and mutable!
        self.next = next

    @property
    def value(self) -> T:
        return self.__value


class LinkedList(Generic[T]):
    def __init__(self, first: Optional[Node[T]], length: int) -> None:
        super().__init__()
        self.__first = first
        self.__len = length

    @staticmethod
    def of(elements: Reversible[T]) -> "LinkedList[T]":
        first: Optional[Node[T]] = None
        length = 0
        for e in reversed(elements):
            first = Node(e, first)
            length += 1
        return LinkedList(first, length)

    def __len__(self) -> int:
        return self.__len

    def __iter__(self) -> Generator[T, None, None]:
        cur = self.__first
        while cur != None:
            yield cur.value
            cur = cur.next

    def extract(self, predicate: Callable[[T], bool]) -> "LinkedList[T]":
        """
        A destructive partition that removes matching elements from this list and puts them into a new one, which is returned.
        """
        cur = self.__first
        self.__first = None
        self.__len = 0

        @dataclass
        class Destination:
            container: LinkedList[T]
            cur: Optional[Node[T]]
        remaining = Destination(
            container=self,
            cur=None,
        )
        extracted = Destination(
            container=LinkedList(None, 0),
            cur=None,
        )
        while cur != None:
            dest = extracted if predicate(cur.value) else remaining
            if dest.cur == None:
                dest.container.__first = cur
            else:
                dest.cur.next = cur
            dest.container.__len += 1
            dest.cur = cur
            cur.next, cur = None, cur.next
        return extracted.container


if __name__ == "__main__":
    import unittest

    class TestLinkedList(unittest.TestCase):
        def test_len(self):
            elems = [i for i in range(5)]
            l = LinkedList.of(elems)
            self.assertEqual(len(elems), len(l))

        def test_iter(self):
            elems = [i for i in range(5)]
            l = LinkedList.of(elems)
            self.assertEqual(elems, list(l))

        def test_extract(self):
            elems = [i for i in range(5)]

            def pred(x):
                return x < 3
            wantExtracted = [x for x in elems if pred(x)]
            wantRemaining = [x for x in elems if not pred(x)]
            l = LinkedList.of(elems)
            extracted = l.extract(pred)
            self.assertEqual(wantExtracted, list(extracted))
            self.assertEqual(wantRemaining, list(l))

    unittest.main()
