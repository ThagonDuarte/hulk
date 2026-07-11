# ruff: noqa: TRY003

import json
from pathlib import Path
from typing import Literal, cast

import click

from ultralytics_dfine import DFINE


@click.command(
    context_settings={"help_option_names": ["-h", "--help"]},
    help="Train D-FINE-S with the official recipe on a YOLO-format dataset.",
)
@click.option("--model", "model_name", default="dfine-s", show_default=True)
@click.option(
    "--data",
    required=True,
    type=click.Path(exists=True, dir_okay=False, path_type=Path),
)
@click.option(
    "--output-dir",
    required=True,
    type=click.Path(file_okay=False, path_type=Path),
)
@click.option("--epochs", default=132, show_default=True, type=int)
@click.option("--transition-epoch", default=None, type=int)
@click.option("--batch", default=32, show_default=True, type=int)
@click.option("--device", default="cuda", show_default=True)
@click.option("--workers", default=4, show_default=True, type=int)
@click.option(
    "--wandb-project",
    default="multi-task-yolo-dfine",
    show_default=True,
)
@click.option(
    "--wandb-mode",
    type=click.Choice(["online", "offline", "disabled"]),
    default="online",
    show_default=True,
)
@click.option("--run-name", default="dfine-s", show_default=True)
@click.option(
    "--resume",
    default=None,
    type=click.Path(exists=True, dir_okay=False, path_type=Path),
)
@click.option("--max-train-batches", default=None, type=int, hidden=True)
@click.option("--max-val-batches", default=None, type=int, hidden=True)
@click.option("--seed", default=0, show_default=True, type=int)
@click.option("--amp/--no-amp", default=True, show_default=True)
def main(
    *,
    model_name: str,
    data: Path,
    output_dir: Path,
    epochs: int,
    transition_epoch: int | None,
    batch: int,
    device: str,
    workers: int,
    wandb_project: str,
    wandb_mode: str,
    run_name: str,
    resume: Path | None,
    max_train_batches: int | None,
    max_val_batches: int | None,
    seed: int,
    amp: bool,
) -> None:
    if model_name != "dfine-s":
        raise click.BadParameter("Only --model dfine-s is supported")
    source: str | Path = resume if resume is not None else model_name
    model = DFINE(source)
    result = model.train(
        data=data,
        output_dir=output_dir,
        epochs=epochs,
        transition_epoch=transition_epoch,
        batch=batch,
        device=device,
        workers=workers,
        wandb_project=wandb_project,
        wandb_mode=cast(
            Literal["online", "offline", "disabled"],
            wandb_mode,
        ),
        name=run_name,
        resume=resume,
        max_train_batches=max_train_batches,
        max_val_batches=max_val_batches,
        seed=seed,
        amp=amp,
    )
    if int(__import__("os").environ.get("RANK", "0")) == 0:
        click.echo(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
