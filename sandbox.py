def envelope_generator(start=1, target=10, slew_rate=512):
    """
    Creates a generator function returning the absolute value differece between
    the start and target voltages over the given duration in ms.
    """
    cv_diff = abs(target - start)
    cur = min(start, target)
    _target = max(start, target)
    slew_rate = max(slew_rate, 1)  # avoid div by zero

    env_progress = 1

    while cur < _target:
        cur = env_progress * cv_diff
        yield cur

result = envelope_generator()
print(result)
