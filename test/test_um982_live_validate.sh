#!/bin/bash
set -euo pipefail

cd "$(dirname "$0")/.."

python3 - <<'PY'
from pathlib import Path

path = Path("tools/um982_live_validate.py")
compile(path.read_text(encoding="utf-8"), str(path), "exec")
PY

help_output="$(python3 tools/um982_live_validate.py --help)"
[[ "$help_output" == *"--port"* ]]
[[ "$help_output" == *"--format {ascii,hybrid,binary}"* ]]
[[ "$help_output" == *"--factory-reset"* ]]
[[ "$help_output" == *"--reset"* ]]
[[ "$help_output" == *"--save-config"* ]]
[[ "$help_output" == *"--com-port"* ]]
[[ "$help_output" == *"--signalgroup-override"* ]]

! rg -n "configure_receiver.sh" tools/um982_live_validate.py >/dev/null 2>&1

normal_plan="$(python3 -c "from tools.um982_live_validate import parse_args, LiveValidator; args=parse_args(['--port','/dev/null']); validator=LiveValidator(args); print('\\n'.join(validator.plan.log_commands))")"
[[ "$normal_plan" == *"LOG PVTSLNA ONTIME 0.2"* ]]
[[ "$normal_plan" == *"LOG BESTNAVA ONTIME 0.2"* ]]
[[ "$normal_plan" != *"BESTSATB"* ]]
[[ "$normal_plan" != *"OBSVMCMPB"* ]]

survey_plan="$(python3 -c "from tools.um982_live_validate import parse_args, LiveValidator; args=parse_args(['--port','/dev/null','--profile','survey','--format','hybrid','--enable-raw']); validator=LiveValidator(args); print('\\n'.join(validator.plan.log_commands))")"
[[ "$survey_plan" == *"LOG PVTSLNA ONTIME 1"* ]]
[[ "$survey_plan" == *"LOG PVTSLNB ONTIME 1"* ]]
[[ "$survey_plan" == *"LOG BESTSATB ONTIME 2"* ]]
[[ "$survey_plan" == *"LOG SATSINFOB ONTIME 2"* ]]
[[ "$survey_plan" == *"LOG OBSVMCMPB ONTIME 5"* ]]

debug_raw_plan="$(python3 -c "from tools.um982_live_validate import parse_args, LiveValidator; args=parse_args(['--port','/dev/null','--profile','debug','--format','hybrid','--enable-raw']); validator=LiveValidator(args); print('\\n'.join(validator.plan.log_commands))")"
[[ "$debug_raw_plan" != *"OBSVMCMPA"* ]]
[[ "$debug_raw_plan" != *"OBSVMCMPB"* ]]

python3 - <<'PY'
import contextlib
import io

from tools.um982_live_validate import (
    build_probe_bauds,
    classify_command_response,
    detect_receiver_model_from_lines,
    extract_version_line,
    LiveValidator,
    parse_args,
)

assert build_probe_bauds(None, 921600, after_factory_reset=True) == [115200, 460800, 921600]
assert build_probe_bauds(921600, 921600, after_factory_reset=False) == [921600, 115200, 460800]
assert classify_command_response("VERSION", ['#VERSIONA,"UM982","R4.10Build15434"']) == "ok"
assert classify_command_response("UNLOGALL", ["unsupported command"]) == "unsupported"
assert classify_command_response("RESET", []) == "no_response"
assert extract_version_line(['#VERSIONA,"UM980","R4.10Build15434"']) is not None
assert detect_receiver_model_from_lines(['#VERSIONA,"UM981","R4.10Build15434"']) == "UM981"

validator = LiveValidator(parse_args(["--port", "/dev/null"]))
validator._handle_unicore_ascii_line(0.0, "#PVTSLNA,foo*00000000")
assert validator.state.message_counters["PVTSLNA"].count == 1
assert validator.state.unicore_ascii_crc_bad == 1
assert "PVTSLNA" not in validator.state.unknown_ascii_types

try:
    with contextlib.redirect_stderr(io.StringIO()):
        parse_args(["--port", "/dev/null", "--factory-reset", "--reset"])
    raise AssertionError("mutually exclusive reset options should fail")
except SystemExit:
    pass
PY

echo "um982_live_validate: OK"
