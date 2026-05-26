import re
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
MAIN_C = ROOT / "Core" / "Src" / "main.c"


def _function_body(source: str, name: str) -> str:
    match = re.search(rf"\b{name}\s*\([^)]*\)\s*\{{", source)
    assert match is not None, f"{name}() not found"
    depth = 1
    index = match.end()
    while index < len(source) and depth > 0:
        char = source[index]
        if char == "{":
            depth += 1
        elif char == "}":
            depth -= 1
        index += 1
    assert depth == 0, f"{name}() body is incomplete"
    return source[match.end(): index - 1]


class StartupSequenceTest(unittest.TestCase):
    def test_app_start_runs_from_startup_task_after_scheduler_starts(self):
        source = MAIN_C.read_text(encoding="utf-8")
        main_body = _function_body(source, "main")
        startup_body = _function_body(source, "StartupTask")

        self.assertNotIn("App_Start(", main_body)
        self.assertIn("xTaskCreate(StartupTask", main_body)
        self.assertIn("vTaskStartScheduler(", main_body)
        self.assertIn("App_Start(", startup_body)
        self.assertIn("vTaskDelete(NULL)", startup_body)


if __name__ == "__main__":
    unittest.main()
