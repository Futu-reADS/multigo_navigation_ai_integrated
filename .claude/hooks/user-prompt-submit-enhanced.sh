#!/bin/bash

# Hook: user-prompt-submit (enhanced with debugging)
# Triggers: When user submits a prompt to Claude Code
# Purpose:
#   1. Log prompts to docs/dev-logs/ (for senior)
#   2. Track session for GitHub commenting (Issue + PR)

# DEBUG: Log hook execution
{
  echo "=== UserPromptSubmit Hook Triggered ==="
  echo "Timestamp: $(date '+%Y-%m-%d %H:%M:%S')"
  echo "Working directory: $(pwd)"
  echo "Script location: $(cd "$(dirname "$0")" && pwd)"
  echo "User: $USER"
} >> /tmp/claude-hook-debug.log 2>&1

# Read stdin once and pass to both hooks
input=$(cat)

# DEBUG: Log input received
{
  echo "Input received (length: ${#input} chars)"
  echo "First 200 chars: ${input:0:200}"
} >> /tmp/claude-hook-debug.log 2>&1

# Get absolute script directory for reliability
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Execute prompt logger (local files)
{
  echo "$input" | node "$SCRIPT_DIR/prompt-logger.js" 2>&1
  echo "✓ Prompt logger executed (exit: $?)"
} >> /tmp/claude-hook-debug.log 2>&1

# Execute GitHub commenter (Issue + PR comments)
{
  echo "$input" | node "$SCRIPT_DIR/github-commenter.js" 2>&1
  echo "✓ GitHub commenter executed (exit: $?)"
} >> /tmp/claude-hook-debug.log 2>&1

echo "=== Hook Complete ===" >> /tmp/claude-hook-debug.log 2>&1
