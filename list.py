from dataclasses import dataclass
from typing import Callable, Generator, Generic, Iterator, List, Optional, Reversible, TypeVar

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


class DoubleBufferedList(Generic[T]):
    def __init__(self, frontbuffer: List[T], backbuffer: List[T], offset: int = 0, length: Optional[int] = None) -> None:
        """A double-buffered list that allows for linear-time partition without re-allocations."""
        self.__frontbuffer = frontbuffer
        self.__backbuffer = backbuffer
        self.__offset = offset
        self.__len = length if length is not None else len(
            frontbuffer) - offset

    @staticmethod
    def of(elements: List[T]) -> "DoubleBufferedList[T]":
        """Adopts the given list, creating a corresponding backbuffer."""
        backbuffer = elements[:]
        return DoubleBufferedList(elements, backbuffer)

    def __len__(self) -> int:
        return self.__len

    def __iter__(self) -> Generator[T, None, None]:
        """Iterate through frontbuffer at time of call.
        Swapping buffers after the initial call has no effect on this."""
        # a generator is only executed on first call,
        # but we need to capture frontbuffer, length and offset immediately,
        # so we return a nested generator
        src = self.__frontbuffer
        it = range(self.__offset, self.__offset + self.__len)

        def g() -> Generator[T, None, None]:
            for i in it:
                yield src[i]
        return g()

    def __setitem__(self, index: int, value: T) -> None:
        if index < 0 or index >= len(self):
            raise IndexError(f"index must be in [0, {len(self)-1}]")
        self.__frontbuffer[self.__offset + index] = value

    def extract(self, predicate: Callable[[T], bool], matches: int) -> "DoubleBufferedList[T]":
        assert 0 <= matches <= len(self)
        old_len = len(self)
        # create new view of just the matches
        extracted = DoubleBufferedList(
            self.__backbuffer, self.__frontbuffer, self.__offset, matches)
        # get iter before swapping buffers, so we read from old frontbuffer
        it = iter(self)
        # swap buffers so we write to old backbuffer
        # (lest we override what we've yet to read)
        self.__frontbuffer, self.__backbuffer = self.__backbuffer, self.__frontbuffer
        # reduce view to only include non-matches
        self.__offset += matches
        self.__len -= matches
        num_kept = 0
        num_extracted = 0
        for elem in it:
            if predicate(elem):
                extracted[num_extracted] = elem
                num_extracted += 1
            else:
                self[num_kept] = elem
                num_kept += 1
        # we must accurately predict the number of extractions so that we can partition correctly
        # (or we'd need some complicated alternating directions)
        assert num_extracted == matches, f"predicted {matches} extractions, but got {num_extracted}"
        assert num_extracted + num_kept == old_len, \
            f"extracted {num_extracted} of {old_len} leaving {num_kept}"
        return extracted


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
