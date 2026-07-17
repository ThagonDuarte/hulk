"""Strict Click numeric parameter types shared by local command-line tools."""

import math

import click


class FiniteFloatRange(click.FloatRange):
    """A float range that also rejects NaN and infinite values."""

    def convert(
        self,
        value: object,
        param: click.Parameter | None,
        ctx: click.Context | None,
    ) -> float:
        converted = super().convert(value, param, ctx)
        if not math.isfinite(converted):
            self.fail(f"{converted!r} is not a finite float", param, ctx)
        return converted
