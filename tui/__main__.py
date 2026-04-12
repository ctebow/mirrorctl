from __future__ import annotations

from tui.app import MirrorctlApp, parse_args


def main(argv: list[str] | None = None) -> None:
    args = parse_args(argv)
    MirrorctlApp(dry_run=args.dry_run).run()


if __name__ == "__main__":
    main()
