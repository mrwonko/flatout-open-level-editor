def ones(n: int) -> int:
    """Returns a bitmask where the lowest n bits are 1."""
    return (1 << n)-1
