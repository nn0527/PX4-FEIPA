"""Sample-based duration confirmation shared by both decision machines."""


def record_hold_sample(condition, now, since):
    """Return the first and latest matching sample timestamps.

    A hold can only be confirmed after at least two distinct sensor samples;
    timer callbacks alone cannot turn one old sample into a confirmation.
    """

    if not condition:
        return None, None
    if since is None:
        since = now
    return since, now


def held_samples(fresh, since, last_sample_at, duration_s):
    return bool(
        fresh
        and since is not None
        and last_sample_at is not None
        and last_sample_at > since
        and (last_sample_at - since).to_sec() >= duration_s
    )

