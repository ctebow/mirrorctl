"""Headless Textual smoke tests (no hardware)."""

from __future__ import annotations

import asyncio

from tui.app import MirrorctlApp


def test_mirrorctl_app_mounts_dry_run():
    async def _run() -> None:
        app = MirrorctlApp(dry_run=True)
        async with app.run_test():
            assert app.fsm_service.is_connected() is False
            assert app.dry_run is True

    asyncio.run(_run())


def test_dry_run_fsm_connect_via_pilot():
    async def _run() -> None:
        app = MirrorctlApp(dry_run=True)
        async with app.run_test() as pilot:
            assert app.fsm_service.is_connected() is False
            await pilot.click("#confirm_arm")
            await pilot.click("#btn_fsm_connect")
            for _ in range(40):
                await pilot.pause(0.05)
                if app.fsm_service.is_connected():
                    break
            assert app.fsm_service.is_connected()

    asyncio.run(_run())
