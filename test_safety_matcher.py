"""Offline checks for matcher_strict.py, using the unchanged commands.py.

Run from the separate test folder: python3 -B test_safety_matcher.py
The 93 policy cases below retain the previously reviewed expected decisions.
These checks do not test audio, Whisper, ROS, or drone execution.
"""
import copy
import itertools
import json
import unittest
import uuid
from datetime import datetime

from commands import COMMANDS, VoiceCommand
from matcher import match_command as baseline_match_command
from matcher import normalize as baseline_normalize
from matcher_strict import make_message, match_command, normalize


# Filled from the existing review matrix, not derived from matcher_strict.
POLICY_CASES = (('V01', 'take off', 'takeoff'),
 ('V02', 'takeoff', 'takeoff'),
 ('V03', 'launch', 'takeoff'),
 ('V04', 'land', 'land'),
 ('V05', 'landing', 'land'),
 ('V06', 'hover', 'hover'),
 ('V07', 'hold', 'hover'),
 ('V08', 'stop', 'hover'),
 ('V09', 'forward', 'forward'),
 ('V10', 'forwards', 'forward'),
 ('V11', 'ahead', 'forward'),
 ('V12', 'backward', 'backward'),
 ('V13', 'backwards', 'backward'),
 ('V14', 'back', 'backward'),
 ('V15', 'reverse', 'backward'),
 ('V16', 'left', 'left'),
 ('V17', 'right', 'right'),
 ('V18', 'up', 'up'),
 ('V19', 'ascend', 'up'),
 ('V20', 'rise', 'up'),
 ('V21', 'down', 'down'),
 ('V22', 'descend', 'down'),
 ('V23', 'turn left', 'turn_left'),
 ('V24', 'rotate left', 'turn_left'),
 ('V25', 'yaw left', 'turn_left'),
 ('V26', 'turn right', 'turn_right'),
 ('V27', 'rotate right', 'turn_right'),
 ('V28', 'yaw right', 'turn_right'),
 ('V29', 'Take off.', 'takeoff'),
 ('V30', 'take-off', 'takeoff'),
 ('V31', '  TAKE   OFF!!!  ', 'takeoff'),
 ('V32', 'please hover', 'hover'),
 ('V33', 'Move forward', 'forward'),
 ('V34', 'Turn left.', 'turn_left'),
 ('I01', 'banana', None),
 ('I02', '', None),
 ('I03', '   ', None),
 ('I04', 'whole world', None),
 ('I05', 'over', None),
 ('I06', 'Holder', None),
 ('I07', 'leftover', None),
 ('I08', 'bright', None),
 ('I09', 'backup', None),
 ('I10', 'takeoff1', None),
 ('I11', 'turnip', None),
 ('I12', '12345', None),
 ('N01', "don't take off.", None),
 ('N02', 'Don’t take off.', None),
 ('N03', 'do not take off', None),
 ('N04', 'never take off', None),
 ('N05', "don't land", None),
 ('N06', "don't move forward", None),
 ('N07', "Don't left", None),
 ('N08', 'not right', None),
 ('N09', "please don't turn left", None),
 ('N10', 'do not stop', None),
 ('N11', 'no launch', None),
 ('N12', 'avoid landing', None),
 ('N13', 'cancel takeoff', None),
 ('N14', 'never go up', None),
 ('A01', "that's all right", None),
 ('A02', "what's up?", None),
 ('A03', 'I left my notebook', None),
 ('A04', 'the land is dry', None),
 ('A05', 'hold on, I am speaking', None),
 ('A06', 'should I take off?', None),
 ('A07', 'if it is safe, land', None),
 ('A08', 'say "take off" without executing it', None),
 ('A09', 'we discussed turning left', None),
 ('A10', 'right now is a good time', None),
 ('A11', 'turn left?', None),
 ('A12', 'rise in temperature', None),
 ('A13', 'done left', None),
 ('M01', 'forward then land', None),
 ('M02', 'land then forward', None),
 ('M03', 'take off and land', None),
 ('M04', 'left or right', None),
 ('M05', 'right and left', None),
 ('M06', 'turn left then forward', None),
 ('M07', 'forward then stop', None),
 ('M08', 'take off stop', None),
 ('M09', 'stop then take off', None),
 ('M10', "don't take off, land instead", None),
 ('M11', 'turn left and turn right', None),
 ('M12', 'up down', None),
 ('D01', 'draw one', None),
 ('D02', 'draw 1', None),
 ('D03', 'draw two', None),
 ('D04', 'draw 2', None),
 ('D05', 'draw digit one', None),
 ('D06', 'draw digit 2', None),
 ('D07', 'draw one then land', None),
 ('D08', 'draw two and move forward', None))

CHECKED_PHRASES = 0

ORIGINAL_CASES = (
    ("Take off.", "takeoff"),
    ("take-off", "takeoff"),
    ("Takeoff!", "takeoff"),
    ("Land", "land"),
    ("please hover", "hover"),
    ("Stop.", "hover"),
    ("Move forward", "forward"),
    ("go back", "backward"),
    ("Left.", "left"),
    ("turn left", "turn_left"),
    ("Turn right.", "turn_right"),
    ("go up", "up"),
    ("descend", "down"),
    ("forward then land", "forward"),
    ("that's all right", "right"),
    ("banana", None),
    ("", None),
)

LIVE_CASES = (
    ("Please hover.", "hover"),
    ("Hold.", "hover"),
    ("Stop.", "hover"),
    ("Hold what?", None),
    ("Do not move forward.", None),
    ("forward then stop.", None),
    ("stop then forward.", None),
    ("Draw one.", None),
    ("draw 2.", None),
    ("draw one then land.", None),
)


class StrictMatcherChecks(unittest.TestCase):
    def check_phrase(self, transcript, expected, label=None):
        global CHECKED_PHRASES
        CHECKED_PHRASES += 1
        with self.subTest(label=label, transcript=transcript):
            command, normalized = match_command(transcript)
            actual = command.name if command else None
            self.assertEqual(actual, expected)
            self.assertEqual(normalized, baseline_normalize(transcript))
            if command is not None:
                self.assertTrue(any(command is original for original in COMMANDS))

    def test_01_reviewed_93_case_policy_matrix(self):
        self.assertEqual(len(POLICY_CASES), 93)
        for case_id, transcript, expected in POLICY_CASES:
            self.check_phrase(transcript, expected, case_id)

    def test_02_supplied_live_transcripts(self):
        for transcript, expected in LIVE_CASES:
            self.check_phrase(transcript, expected)

    def test_03_original_17_cases_with_two_deliberate_policy_changes(self):
        changed = {"forward then land", "that's all right"}
        for transcript, original_expected in ORIGINAL_CASES:
            with self.subTest(baseline_transcript=transcript):
                original, _ = baseline_match_command(transcript)
                self.assertEqual(original.name if original else None, original_expected)
            expected = None if transcript in changed else original_expected
            self.check_phrase(transcript, expected)

    def test_04_all_existing_aliases_and_polite_forms(self):
        for command in COMMANDS:
            for alias in command.phrases:
                for transcript in (
                    alias,
                    "please " + alias,
                    alias + " please",
                    "please " + alias + " please",
                ):
                    self.check_phrase(transcript, command.name)

    def test_05_directional_go_and_move_forms(self):
        direction_names = {"forward", "backward", "left", "right", "up", "down"}
        for command in COMMANDS:
            if command.name not in direction_names:
                continue
            for alias in command.phrases:
                for verb in ("go", "move"):
                    for prefix, suffix in (("", ""), ("please ", ""), ("", " please"), ("please ", " please")):
                        self.check_phrase(prefix + verb + " " + alias + suffix, command.name)

    def test_06_case_whitespace_and_approved_punctuation(self):
        for command in COMMANDS:
            for alias in command.phrases:
                for transcript in (
                    alias.upper(),
                    "  " + alias.upper().replace(" ", "   ") + "!!!  ",
                    alias + ".",
                    "please\t" + alias,
                ):
                    self.check_phrase(transcript, command.name)
        self.check_phrase("Please take-off.", "takeoff")

    def test_07_negated_or_cancelled_requests(self):
        templates = (
            "do not {}", "don't {}", "Don’t {}", "never {}", "not {}",
            "no {}", "avoid {}", "cancel {}", "please do not {}",
            "without {}", "do NOT {} please", "dont {}",
        )
        for command in COMMANDS:
            for alias in command.phrases:
                for template in templates:
                    self.check_phrase(template.format(alias), None)

    def test_08_every_alias_pair_is_rejected(self):
        aliases = [alias for command in COMMANDS for alias in command.phrases]
        for first, second in itertools.product(aliases, repeat=2):
            for connector in (" then ", " and ", " or ", " ", ", "):
                self.check_phrase(first + connector + second, None)

    def test_09_extra_unsupported_instructions_are_rejected(self):
        templates = (
            "draw one then {}", "draw two and {}", "if safe {}",
            "I said {}", "say {}", "{} for ten seconds",
            "{} 10 meters", "{} now", "{} but don't execute",
            "when I say go {}", "do you mean {}", "can you {}",
        )
        for command in COMMANDS:
            for alias in command.phrases:
                for template in templates:
                    self.check_phrase(template.format(alias), None)

    def test_10_no_fallback_for_conversation_or_drawing(self):
        for transcript in (
            "Hold what?", "hold on", "that's all right", "I left my bag",
            "what's up", "the land is dry", "right now", "done left",
            "draw one", "draw two", "draw 1", "draw 2", "draw digit one",
            "please draw one", "move in a circle", "go take off",
            "move land", "go hover", "please please hover",
        ):
            self.check_phrase(transcript, None)

    def test_11_unapproved_syntax_is_not_erased_into_a_command(self):
        for transcript in (
            "forward?", '"forward"', "'forward'", "(forward)", "forward/",
            "take/off", "take--off", "forward. stop", "forward;stop",
            "forward\n", "forward\rstop", "take\noff", "take\toff?",
            "forward🚫", "forward\u200b", "take\u2011off", "take\u00a0off",
            "forward0", "forward 2", "-forward", "forward-", ".forward",
            "\u00a0forward", "forward\u00a0", "\vforward", "forward\f",
            "ta\u212aeoff", "\nforward", "forward\tthen\tstop", "\u200bhover",
        ):
            self.check_phrase(transcript, None)

    def test_12_duplicate_aliases_cannot_silently_choose_a_command(self):
        first = VoiceCommand("first", ("hover",), {"marker": 1})
        second = VoiceCommand("second", ("hover",), {"marker": 2})
        for transcript in ("hover", "please hover", "hover please"):
            with self.subTest(transcript=transcript):
                command, _ = match_command(transcript, [first, second])
                self.assertIsNone(command)
        command, _ = match_command("hover", [])
        self.assertIsNone(command)

    def test_13_json_contract_and_existing_command_parameters(self):
        keys = {"id", "timestamp", "transcript", "normalized", "status", "command", "params", "audio_rms"}
        for transcript, expected in LIVE_CASES:
            with self.subTest(transcript=transcript):
                command, normalized = match_command(transcript)
                message = make_message(transcript, normalized, command, rms=0.01044)
                self.assertEqual(set(message), keys)
                self.assertEqual(message["transcript"], transcript)
                self.assertEqual(message["normalized"], baseline_normalize(transcript))
                self.assertEqual(message["status"], "accepted" if expected else "rejected")
                self.assertEqual(message["command"], expected)
                self.assertEqual(message["params"], dict(command.params) if command else {})
                self.assertEqual(message["audio_rms"], 0.0104)
                self.assertEqual(str(uuid.UUID(message["id"])), message["id"])
                self.assertIsNotNone(datetime.fromisoformat(message["timestamp"]).utcoffset())
                self.assertEqual(json.loads(json.dumps(message)), message)
        command, normalized = match_command("hover")
        self.assertIsNone(make_message("hover", normalized, command)["audio_rms"])

    def test_14_matching_does_not_mutate_command_definitions(self):
        before = copy.deepcopy([(cmd.name, cmd.phrases, cmd.params) for cmd in COMMANDS])
        for transcript, _ in LIVE_CASES:
            match_command(transcript)
        self.assertEqual([(cmd.name, cmd.phrases, cmd.params) for cmd in COMMANDS], before)

    def test_15_normalization_contract_matches_original(self):
        for transcript in ("Don’t take-off.", "  Please   HOVER! ", "draw 2.", "", "left/right"):
            with self.subTest(transcript=transcript):
                self.assertEqual(normalize(transcript), baseline_normalize(transcript))


if __name__ == "__main__":
    import sys

    suite = unittest.defaultTestLoader.loadTestsFromTestCase(StrictMatcherChecks)
    result = unittest.TextTestRunner(verbosity=2).run(suite)
    print("\nChecked {} phrase expectations; audio/ROS/flight not exercised.".format(CHECKED_PHRASES), flush=True)
    sys.exit(0 if result.wasSuccessful() else 1)

