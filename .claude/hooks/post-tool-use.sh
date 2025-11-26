#!/bin/bash

# Hook: post-tool-use (enhanced with debugging)
# Triggers: After each tool execution
# Purpose: Post GitHub comments (Issue + PR) summarizing Claude Code sessions

# DEBUG: Log hook execution
{
  echo "=== PostToolUse Hook Triggered ==="
  echo "Timestamp: $(date '+%Y-%m-%d %H:%M:%S')"
  echo "Working directory: $(pwd)"
  echo "Arguments: $@"
} >> /tmp/claude-hook-debug.log 2>&1

# Get absolute script directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Execute the GitHub commenter with stdin
{
  node "$SCRIPT_DIR/github-commenter.js" 2>&1
  echo "✓ GitHub commenter executed (exit: $?)"
} >> /tmp/claude-hook-debug.log 2>&1

echo "=== Hook Complete ===" >> /tmp/claude-hook-debug.log 2>&1
