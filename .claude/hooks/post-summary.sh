#!/bin/bash

###############################################################################
# Post Summary to GitHub
#
# Posts formatted comments to Issues (work progress) and PRs (updates)
#
# Usage (option 1 - Interactive):
#   ./.claude/hooks/post-summary.sh
#
# Usage (option 2 - AUTO with priority chain, 2 params):
#   ./.claude/hooks/post-summary.sh \
#     "Your formatted request: Add JSDoc comments" \
#     "Achievement: Added docs to 9 files, 460+ lines, all tests passing"
#   → Automatically detects prompt from:
#     1. .claude/prompt-history.json (fresh session, hook captured)
#     2. .claude-prompt-issue-{NUM}.md (workflow file, survives resumption)
#
# Usage (option 3 - MANUAL override, 3 params):
#   ./.claude/hooks/post-summary.sh \
#     "add comments to codebase" \
#     "Add comprehensive JSDoc comments throughout the codebase" \
#     "Achievement: Added docs to 9 files, 460+ lines, all tests passing"
#   → Uses provided actual prompt (highest priority, overrides auto-detection)
#
# Format:
#   - Issue comments: "Response #N" (work-in-progress updates)
#   - PR comments: "Update #N" (PR iteration updates)
#
# Posting Rules:
#   - If PR exists: Posts to PR ONLY (PR takes precedence)
#   - If no PR: Posts to Issue
#   - Never posts to both Issue and PR simultaneously
#   - Does NOT mention @claude to avoid triggering GitHub AI auto-implementation
#
# Enable/Disable:
#   - To disable: export DISABLE_AUTO_COMMENT=true
#   - To enable: unset DISABLE_AUTO_COMMENT (or export DISABLE_AUTO_COMMENT=false)
#
# Debug Logging:
#   - To enable: export DEBUG_POST_SUMMARY=true
#   - Logs to: .claude/post-summary-debug.log
#   - Shows: prompt detection flow, sources checked, results
#
# Writing Good Summaries:
#   See COMMENT-WRITING-GUIDE.md for detailed guidelines and examples
#
#   Comment Structure:
#   - ACTUAL_PROMPT (optional): Raw user input, verbatim
#     * Shows <= 3 lines: Displayed directly
#     * Shows > 3 lines: Collapsed with "View full prompt" button
#   - REQUEST: Formatted/contextualized description of task
#   - RESPONSE/CHANGES MADE: What was achieved
#
#   Quick Tips:
#   - ACTUAL_PROMPT: Verbatim user input (e.g., "add comments to codebase")
#   - USER_PROMPT: Be specific, include context
#   - ACHIEVEMENT: Use structured format with multiple paragraphs
#     * Explain WHAT was done (specific changes)
#     * Explain WHY it was done (reasoning, bugs fixed)
#     * List files/functions modified
#     * Mention testing done
#     * Note acceptance criteria addressed
#     * Include next steps if work continues
#
#   Bad Example:  "Fixed bug"
#   Good Example: "Fixed coverage timestamp bug that caused stale detection.
#                  Changed from commit-based to age-based check (24h window).
#                  Modified parse-coverage.sh:54-61. Tested with npm test."
###############################################################################

set -euo pipefail  # Fail on errors, undefined vars, pipe failures
IFS=$'\n\t'        # Prevent word splitting issues

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

# Source coverage parsing helper
source "$PROJECT_ROOT/scripts/parse-coverage.sh"

# Debug logging (optional, controlled by DEBUG_POST_SUMMARY env var)
DEBUG_LOG=".claude/post-summary-debug.log"
log_debug() {
  if [ "${DEBUG_POST_SUMMARY:-false}" = "true" ]; then
    echo "[$(date '+%Y-%m-%d %H:%M:%S')] $1" >> "$DEBUG_LOG"
  fi
}

###############################################################################
# Security Functions
###############################################################################

# Sanitize input to prevent command injection
# Usage: SAFE_VAR=$(sanitize_input "$UNSAFE_VAR")
sanitize_input() {
  local input="$1"
  # Escape shell metacharacters: $ ` \ ! ; | & ( ) < >
  # Using printf instead of echo for safety with special characters
  printf '%s' "$input" | sed 's/[$`\\!;|&()<>]/\\&/g'
}

# Sanitize for GitHub to prevent unwanted @mentions and injections
# Usage: SAFE_COMMENT=$(sanitize_for_github "$COMMENT")
sanitize_for_github() {
  local input="$1"
  # Escape all @mentions (add space after @)
  # Also escape shell metacharacters
  echo "$input" | sed 's/@/@ /g; s/[$`]/\\&/g'
}

# Check if auto-commenting is disabled
if [ "${DISABLE_AUTO_COMMENT:-false}" = "true" ]; then
  echo "ℹ️  Auto-commenting is disabled (DISABLE_AUTO_COMMENT=true)"
  echo "To enable: unset DISABLE_AUTO_COMMENT"
  exit 0
fi

# Get branch info
BRANCH=$(git branch --show-current 2>/dev/null || echo "")
ISSUE_NUM=$(echo "$BRANCH" | grep -oP '\d+' | head -1 || echo "")
PR_NUM=$(gh pr list --head "$BRANCH" --json number --jq '.[0].number' 2>/dev/null || echo "")

if [ -z "$ISSUE_NUM" ] && [ -z "$PR_NUM" ]; then
  echo "❌ No issue or PR found (branch: $BRANCH)"
  echo "   Branch name should contain issue number (e.g., feature/21-description)"
  exit 1
fi

# Auto-generate REQUEST and ACHIEVEMENT from git commit
# Takes commit SHA as parameter, returns formatted strings
auto_generate_descriptions() {
  local commit_sha="$1"

  # Extract commit message parts
  local commit_subject=$(git log -1 --format=%s "$commit_sha" 2>/dev/null || echo "")
  local commit_body=$(git log -1 --format=%b "$commit_sha" 2>/dev/null || echo "")

  # Generate REQUEST from commit subject (remove issue number)
  AUTO_REQUEST=$(echo "$commit_subject" | sed 's/ (#[0-9]\+)$//')

  # Generate ACHIEVEMENT from commit body + diff stats
  local files_changed=$(git diff --name-only "${commit_sha}~1..$commit_sha" 2>/dev/null | wc -l | tr -d ' ')
  local diff_stat=$(git diff --stat "${commit_sha}~1..$commit_sha" 2>/dev/null | tail -1)

  if [ -n "$commit_body" ]; then
    AUTO_ACHIEVEMENT="$commit_body

**Changes:** Modified $files_changed files
$diff_stat"
  else
    AUTO_ACHIEVEMENT="**Changes:** Modified $files_changed files
$diff_stat"
  fi

  log_debug "Auto-generated REQUEST: $AUTO_REQUEST"
  log_debug "Auto-generated ACHIEVEMENT: ${#AUTO_ACHIEVEMENT} chars"
}

# Get user input
# Supports 4 formats:
#   0 params (FULL AUTO): post-summary.sh
#      → Automatically generates everything from latest commit
#      → Detects prompt from multiple sources
#      → Perfect for post-commit hook (zero manual work)
#   2 params (SEMI-AUTO): post-summary.sh "request" "achievement"
#      → Automatically detects prompt from multiple sources
#      → Manual REQUEST and ACHIEVEMENT
#   3 params (MANUAL override): post-summary.sh "actual_prompt" "request" "achievement"
#      → Uses provided actual_prompt (highest priority)
#   "interactive" (INTERACTIVE): post-summary.sh interactive
#      → Prompts for all inputs
#
# Prompt Source Priority (automatic in 0-param and 2-param modes):
#   1. Explicit 3rd parameter (manual override)
#   2. .claude/prompt-history.json with timestamp matching (fresh session, hook captured)
#   3. .claude-prompt-issue-{NUM}.md (workflow file, survives resumption)
#   4. Skip Actual Prompt section (no source found)

if [ "${1:-}" = "interactive" ]; then
  # Interactive mode - prompt for everything
  echo "What was the actual prompt? (press Enter to skip)"
  read -r ACTUAL_PROMPT_RAW
  ACTUAL_PROMPT=$(sanitize_input "$ACTUAL_PROMPT_RAW")
  echo "What did you ask Claude to do? (formatted request)"
  read -r USER_PROMPT
  echo "What was achieved?"
  read -r ACHIEVEMENT
  PROMPT_SOURCE="interactive"

elif [ -n "${3:-}" ]; then
  # 3 parameters: manual override with explicit actual prompt (PRIORITY 1)
  ACTUAL_PROMPT=$(sanitize_input "$1")
  USER_PROMPT="$2"
  ACHIEVEMENT="$3"
  PROMPT_SOURCE="manual-override"

elif [ -n "${1:-}" ]; then
  # 2 parameters: SEMI-AUTO MODE with priority chain
  USER_PROMPT="$1"
  ACHIEVEMENT="$2"
  ACTUAL_PROMPT=""
  PROMPT_SOURCE="none"

  log_debug "=== Prompt Detection Start (SEMI-AUTO) ==="
  log_debug "Branch: $BRANCH"
  log_debug "Issue Number: $ISSUE_NUM"

  # PRIORITY 2: Try to read prompt from history with timestamp matching
  HISTORY_FILE=".claude/prompt-history.json"
  log_debug "Checking prompt history: $HISTORY_FILE"
  if [ -f "$HISTORY_FILE" ]; then
    log_debug "History file exists"
    if command -v jq &> /dev/null; then
      # Get commit timestamp for matching
      COMMIT_TIME=$(git log -1 --format=%ct 2>/dev/null || echo "0")
      log_debug "Commit timestamp: $COMMIT_TIME"

      # Find prompt within 10 minutes before commit, matching issue number
      MATCHED_PROMPT=$(jq -r --arg ct "$COMMIT_TIME" --arg issue "$ISSUE_NUM" '
        .prompts[] |
        select(.issueNumber == $issue) |
        select((.timestamp | tonumber) <= ($ct | tonumber)) |
        select(($ct | tonumber) - (.timestamp | tonumber) < 600) |
        .prompt
      ' "$HISTORY_FILE" 2>/dev/null | tail -1)

      if [ -n "$MATCHED_PROMPT" ]; then
        ACTUAL_PROMPT=$(sanitize_input "$MATCHED_PROMPT")
        PROMPT_SOURCE="prompt-history-matched"
        log_debug "SUCCESS: Matched prompt by timestamp (${#ACTUAL_PROMPT} chars, within 10min)"
        echo "✓ Loaded prompt from history (timestamp matched)"
      else
        log_debug "No matching prompt found by timestamp"
        echo "ℹ️  No recent prompt in history (checked within 10min)"
      fi
    else
      log_debug "jq not available (install with: apt-get install jq)"
      echo "⚠️  jq not installed, cannot read prompt history (install with: apt-get install jq)"
    fi
  else
    log_debug "History file not found"
  fi

  # PRIORITY 3: Check for workflow prompt file (survives resumption)
  if [ -z "$ACTUAL_PROMPT" ] && [ -n "$ISSUE_NUM" ]; then
    WORKFLOW_PROMPT_FILE=".claude-prompt-issue-${ISSUE_NUM}.md"
    log_debug "Checking workflow file: $WORKFLOW_PROMPT_FILE"
    if [ -f "$WORKFLOW_PROMPT_FILE" ]; then
      # Security: Validate file size before reading (1MB limit)
      FILE_SIZE=$(wc -c < "$WORKFLOW_PROMPT_FILE" 2>/dev/null | tr -d ' ' || echo 0)
      if [ "$FILE_SIZE" -gt 1048576 ]; then  # 1MB = 1048576 bytes
        log_debug "Workflow file too large: ${FILE_SIZE} bytes (limit: 1MB)"
        echo "⚠️  Workflow file too large (${FILE_SIZE} bytes), skipping"
      else
        ACTUAL_PROMPT=$(sanitize_input "$(cat "$WORKFLOW_PROMPT_FILE")")
        PROMPT_SOURCE="workflow-file"
        log_debug "SUCCESS: Loaded from workflow file (${#ACTUAL_PROMPT} chars)"
        echo "✓ Loaded prompt from workflow file: $WORKFLOW_PROMPT_FILE"
      fi
    else
      log_debug "Workflow file not found"
    fi
  elif [ -z "$ACTUAL_PROMPT" ]; then
    log_debug "Skipping workflow file check (no issue number)"
  fi

  # PRIORITY 4: No source found (will skip Actual Prompt section)
  if [ -z "$ACTUAL_PROMPT" ]; then
    log_debug "RESULT: No prompt found (all sources exhausted)"
    echo "ℹ️  No prompt found (tried history + workflow file)"
    PROMPT_SOURCE="none"
  fi

  log_debug "=== Prompt Detection End (source: $PROMPT_SOURCE) ==="

else
  # 0 parameters: FULL AUTO MODE - generate everything from git
  log_debug "=== FULL AUTO MODE ==="

  # Get latest commit
  LATEST_COMMIT=$(git log -1 --format=%H 2>/dev/null || echo "")
  if [ -z "$LATEST_COMMIT" ]; then
    echo "❌ No commits found"
    exit 1
  fi

  log_debug "Latest commit: $LATEST_COMMIT"

  # Auto-generate REQUEST and ACHIEVEMENT
  auto_generate_descriptions "$LATEST_COMMIT"
  USER_PROMPT="$AUTO_REQUEST"
  ACHIEVEMENT="$AUTO_ACHIEVEMENT"

  echo "✓ Auto-generated from commit: ${LATEST_COMMIT:0:7}"

  # Now detect prompt with timestamp matching
  ACTUAL_PROMPT=""
  PROMPT_SOURCE="none"

  log_debug "=== Prompt Detection Start (FULL AUTO) ==="
  log_debug "Branch: $BRANCH"
  log_debug "Issue Number: $ISSUE_NUM"

  # PRIORITY 2: Try to read prompt from history with timestamp matching
  HISTORY_FILE=".claude/prompt-history.json"
  log_debug "Checking prompt history: $HISTORY_FILE"
  if [ -f "$HISTORY_FILE" ]; then
    log_debug "History file exists"
    if command -v jq &> /dev/null; then
      # Get commit timestamp for matching
      COMMIT_TIME=$(git log -1 --format=%ct 2>/dev/null || echo "0")
      log_debug "Commit timestamp: $COMMIT_TIME"

      # Find prompt within 10 minutes before commit, matching issue number
      MATCHED_PROMPT=$(jq -r --arg ct "$COMMIT_TIME" --arg issue "$ISSUE_NUM" '
        .prompts[] |
        select(.issueNumber == $issue) |
        select((.timestamp | tonumber) <= ($ct | tonumber)) |
        select(($ct | tonumber) - (.timestamp | tonumber) < 600) |
        .prompt
      ' "$HISTORY_FILE" 2>/dev/null | tail -1)

      if [ -n "$MATCHED_PROMPT" ]; then
        ACTUAL_PROMPT=$(sanitize_input "$MATCHED_PROMPT")
        PROMPT_SOURCE="prompt-history-matched"
        log_debug "SUCCESS: Matched prompt by timestamp (${#ACTUAL_PROMPT} chars, within 10min)"
        echo "✓ Loaded prompt from history (timestamp matched)"
      else
        log_debug "No matching prompt found by timestamp"
        echo "ℹ️  No recent prompt in history (checked within 10min)"
      fi
    else
      log_debug "jq not available (install with: apt-get install jq)"
      echo "⚠️  jq not installed, cannot read prompt history"
    fi
  else
    log_debug "History file not found"
  fi

  # PRIORITY 3: Check for workflow prompt file (survives resumption)
  if [ -z "$ACTUAL_PROMPT" ] && [ -n "$ISSUE_NUM" ]; then
    WORKFLOW_PROMPT_FILE=".claude-prompt-issue-${ISSUE_NUM}.md"
    log_debug "Checking workflow file: $WORKFLOW_PROMPT_FILE"
    if [ -f "$WORKFLOW_PROMPT_FILE" ]; then
      # Security: Validate file size before reading (1MB limit)
      FILE_SIZE=$(wc -c < "$WORKFLOW_PROMPT_FILE" 2>/dev/null | tr -d ' ' || echo 0)
      if [ "$FILE_SIZE" -gt 1048576 ]; then  # 1MB = 1048576 bytes
        log_debug "Workflow file too large: ${FILE_SIZE} bytes (limit: 1MB)"
        echo "⚠️  Workflow file too large (${FILE_SIZE} bytes), skipping"
      else
        ACTUAL_PROMPT=$(sanitize_input "$(cat "$WORKFLOW_PROMPT_FILE")")
        PROMPT_SOURCE="workflow-file"
        log_debug "SUCCESS: Loaded from workflow file (${#ACTUAL_PROMPT} chars)"
        echo "✓ Loaded prompt from workflow file: $WORKFLOW_PROMPT_FILE"
      fi
    else
      log_debug "Workflow file not found"
    fi
  elif [ -z "$ACTUAL_PROMPT" ]; then
    log_debug "Skipping workflow file check (no issue number)"
  fi

  # PRIORITY 4: No source found (will skip Actual Prompt section)
  if [ -z "$ACTUAL_PROMPT" ]; then
    log_debug "RESULT: No prompt found (all sources exhausted)"
    echo "ℹ️  No prompt found (tried history + workflow file)"
    PROMPT_SOURCE="none"
  fi

  log_debug "=== Prompt Detection End (source: $PROMPT_SOURCE) ==="
fi

# Function to format Actual Prompt section
# Returns formatted section or empty string if no actual prompt
format_actual_prompt() {
  local actual_prompt="$1"

  # If no actual prompt provided, return empty
  [ -z "$actual_prompt" ] && return

  # Count lines (newlines + 1)
  local line_count=$(echo "$actual_prompt" | grep -c $'\n')
  line_count=$((line_count + 1))

  # If 3 or fewer lines, show directly
  if [ $line_count -le 3 ]; then
    echo "### Actual Prompt

${actual_prompt}

---

"
  else
    # More than 3 lines, use collapsible details
    echo "### Actual Prompt

<details>
<summary>View full prompt (${line_count} lines)</summary>

${actual_prompt}

</details>

---

"
  fi
}

# Get commits and files
BASE_BRANCH="main"
COMMITS=$(git log --oneline ${BASE_BRANCH}..HEAD 2>/dev/null | head -10)
# Escape ALL @mentions in commit messages to prevent triggering unwanted GitHub notifications
COMMITS=$(sanitize_for_github "$COMMITS")
COMMIT_COUNT=$(echo "$COMMITS" | wc -l)

# Get files changed in latest commit only (incremental)
LATEST_COMMIT_FILES=$(git diff --name-only HEAD~1..HEAD 2>/dev/null || git diff --name-only --cached 2>/dev/null)
LATEST_FILE_COUNT=$(echo "$LATEST_COMMIT_FILES" | grep -v '^$' | wc -l)

# Get all files changed in branch (cumulative)
CHANGED_FILES=$(git diff --name-only ${BASE_BRANCH}...HEAD 2>/dev/null)
FILE_COUNT=$(echo "$CHANGED_FILES" | grep -v '^$' | wc -l)

# Timing
FIRST_COMMIT=$(git log ${BASE_BRANCH}..HEAD --reverse --format=%ct 2>/dev/null | head -1)
LAST_COMMIT=$(git log -1 --format=%ct)
if [ -n "$FIRST_COMMIT" ]; then
  DURATION=$(( (LAST_COMMIT - FIRST_COMMIT) / 60 ))
  DURATION_STR="${DURATION}m"
else
  DURATION_STR="30m"
fi

TIMESTAMP=$(date '+%Y-%m-%d %H:%M')

# Response/Update number tracking
SESSION_FILE="$SCRIPT_DIR/../session-counter.json"
[ ! -f "$SESSION_FILE" ] && echo '{}' > "$SESSION_FILE"

# Separate counters for Issue and PR
ISSUE_KEY="issue-${ISSUE_NUM}"
PR_KEY="pr-${PR_NUM}"

# Build Issue comment (if issue exists)
if [ -n "$ISSUE_NUM" ]; then
  ISSUE_RESPONSE_NUM=$(jq -r ".[\"$ISSUE_KEY\"] // 0" "$SESSION_FILE")
  ISSUE_RESPONSE_NUM=$((ISSUE_RESPONSE_NUM + 1))
  jq ".[\"$ISSUE_KEY\"] = $ISSUE_RESPONSE_NUM" "$SESSION_FILE" > "$SESSION_FILE.tmp" && mv "$SESSION_FILE.tmp" "$SESSION_FILE"

  ISSUE_COMMENT="## ClaudeCode Response #${ISSUE_RESPONSE_NUM}

\`\`\`
Time: ${TIMESTAMP}
\`\`\`

---

"

  # Add Actual Prompt section if provided
  ACTUAL_PROMPT_SECTION=$(format_actual_prompt "$ACTUAL_PROMPT")
  if [ -n "$ACTUAL_PROMPT_SECTION" ]; then
    ISSUE_COMMENT="${ISSUE_COMMENT}${ACTUAL_PROMPT_SECTION}
"
  fi

  ISSUE_COMMENT="${ISSUE_COMMENT}### Request

${USER_PROMPT}

---

### Response

${ACHIEVEMENT}

---

"

  # Add coverage section if available
  COVERAGE_KEY="coverage-issue-${ISSUE_NUM}"
  COVERAGE_SECTION=$(parse_coverage_section "$SESSION_FILE" "$COVERAGE_KEY" "$ISSUE_RESPONSE_NUM")
  [ -n "$COVERAGE_SECTION" ] && ISSUE_COMMENT="${ISSUE_COMMENT}${COVERAGE_SECTION}"

  ISSUE_COMMENT="${ISSUE_COMMENT}
### Files Changed in this Response

<details>
<summary>${LATEST_FILE_COUNT} files</summary>

"

  # Add latest commit files to issue comment
  if [ $LATEST_FILE_COUNT -gt 0 ]; then
    while IFS= read -r file; do
      if [ -n "$file" ]; then
        ISSUE_COMMENT="${ISSUE_COMMENT}- \`${file}\`
"
      fi
    done < <(echo "$LATEST_COMMIT_FILES")
  fi

  ISSUE_COMMENT="${ISSUE_COMMENT}
</details>

---

### All Files Changed in this Branch

<details>
<summary>Total: ${FILE_COUNT} files</summary>

"

  # Add all branch files to issue comment
  if [ $FILE_COUNT -gt 0 ]; then
    while IFS= read -r file; do
      if [ -n "$file" ]; then
        ISSUE_COMMENT="${ISSUE_COMMENT}- \`${file}\`
"
      fi
    done < <(echo "$CHANGED_FILES" | head -15)

    if [ $FILE_COUNT -gt 15 ]; then
      ISSUE_COMMENT="${ISSUE_COMMENT}
... and $((FILE_COUNT - 15)) more files"
    fi
  fi

  ISSUE_COMMENT="${ISSUE_COMMENT}

</details>

---

### Commits in this Branch

\`\`\`
${COMMITS}
\`\`\`

---

**Status:** Implementation completed and committed locally (not pushed yet)

<sub>Response #${ISSUE_RESPONSE_NUM} - Auto-generated by ClaudeCode</sub>
"
fi

# Build PR comment (if PR exists)
if [ -n "$PR_NUM" ]; then
  PR_UPDATE_NUM=$(jq -r ".[\"$PR_KEY\"] // 0" "$SESSION_FILE")
  PR_UPDATE_NUM=$((PR_UPDATE_NUM + 1))
  jq ".[\"$PR_KEY\"] = $PR_UPDATE_NUM" "$SESSION_FILE" > "$SESSION_FILE.tmp" && mv "$SESSION_FILE.tmp" "$SESSION_FILE"

  PR_COMMENT="## ClaudeCode Update #${PR_UPDATE_NUM}

\`\`\`
Time: ${TIMESTAMP}
\`\`\`

---

"

  # Add Actual Prompt section if provided
  ACTUAL_PROMPT_SECTION=$(format_actual_prompt "$ACTUAL_PROMPT")
  if [ -n "$ACTUAL_PROMPT_SECTION" ]; then
    PR_COMMENT="${PR_COMMENT}${ACTUAL_PROMPT_SECTION}
"
  fi

  PR_COMMENT="${PR_COMMENT}### Request

${USER_PROMPT}

---

### Changes Made

${ACHIEVEMENT}

---

"

  # Add coverage section if available
  COVERAGE_KEY="coverage-pr-${PR_NUM}"
  COVERAGE_SECTION=$(parse_coverage_section "$SESSION_FILE" "$COVERAGE_KEY" "$PR_UPDATE_NUM")
  [ -n "$COVERAGE_SECTION" ] && PR_COMMENT="${PR_COMMENT}${COVERAGE_SECTION}"

  PR_COMMENT="${PR_COMMENT}
### Files Changed in this Update

<details>
<summary>${LATEST_FILE_COUNT} files</summary>

"

  # Add latest commit files to PR comment
  if [ $LATEST_FILE_COUNT -gt 0 ]; then
    while IFS= read -r file; do
      if [ -n "$file" ]; then
        PR_COMMENT="${PR_COMMENT}- \`${file}\`
"
      fi
    done < <(echo "$LATEST_COMMIT_FILES")
  fi

  PR_COMMENT="${PR_COMMENT}
</details>

---

### All Files Changed in this Branch

<details>
<summary>Total: ${FILE_COUNT} files</summary>

"

  # Add all branch files to PR comment
  if [ $FILE_COUNT -gt 0 ]; then
    while IFS= read -r file; do
      if [ -n "$file" ]; then
        PR_COMMENT="${PR_COMMENT}- \`${file}\`
"
      fi
    done < <(echo "$CHANGED_FILES" | head -15)

    if [ $FILE_COUNT -gt 15 ]; then
      PR_COMMENT="${PR_COMMENT}
... and $((FILE_COUNT - 15)) more files"
    fi
  fi

  PR_COMMENT="${PR_COMMENT}

</details>

---

### All Commits in this Branch

\`\`\`
${COMMITS}
\`\`\`

---

**Status:** Changes pushed to PR and ready for review

<sub>Update #${PR_UPDATE_NUM} - Auto-generated by ClaudeCode</sub>
"
fi

# Function to post comment with retry logic
# Parameters: $1 = target type ("pr" or "issue"), $2 = target number, $3 = comment body
post_with_retry() {
  local target_type="$1"
  local target_num="$2"
  local comment_body="$3"
  local max_retries=3
  local retry_delay=2

  for ((i=1; i<=max_retries; i++)); do
    log_debug "Posting attempt $i/$max_retries to $target_type #$target_num"

    if [ "$target_type" = "pr" ]; then
      if echo "$comment_body" | gh pr comment "$target_num" --body-file - 2>&1; then
        return 0  # Success
      fi
    else
      if echo "$comment_body" | gh issue comment "$target_num" --body-file - 2>&1; then
        return 0  # Success
      fi
    fi

    # Failed, retry if not last attempt
    if [ $i -lt $max_retries ]; then
      echo "⚠️  Post failed (attempt $i/$max_retries), retrying in ${retry_delay}s..."
      log_debug "Retry after ${retry_delay}s delay"
      sleep $retry_delay
      retry_delay=$((retry_delay * 2))  # Exponential backoff
    fi
  done

  # All retries failed
  echo "❌ Failed to post after $max_retries attempts"
  log_debug "All retry attempts exhausted"
  return 1
}

# Post comments - PR takes precedence over Issue
# Once a PR is created, all updates go to PR only (not both)
if [ -n "$PR_NUM" ]; then
  if post_with_retry "pr" "$PR_NUM" "$PR_COMMENT"; then
    echo "✓ Posted to PR #${PR_NUM} (Update #${PR_UPDATE_NUM})"
  else
    echo "❌ Could not post to PR #${PR_NUM}"
    echo "   Comment saved to: .claude/failed-comment-pr-${PR_NUM}.txt"
    echo "$PR_COMMENT" > ".claude/failed-comment-pr-${PR_NUM}.txt"
    exit 1
  fi
elif [ -n "$ISSUE_NUM" ]; then
  if post_with_retry "issue" "$ISSUE_NUM" "$ISSUE_COMMENT"; then
    echo "✓ Posted to Issue #${ISSUE_NUM} (Response #${ISSUE_RESPONSE_NUM})"
  else
    echo "❌ Could not post to Issue #${ISSUE_NUM}"
    echo "   Comment saved to: .claude/failed-comment-issue-${ISSUE_NUM}.txt"
    echo "$ISSUE_COMMENT" > ".claude/failed-comment-issue-${ISSUE_NUM}.txt"
    exit 1
  fi
else
  echo "❌ No issue or PR found to post to"
  exit 1
fi

echo "✅ Done!"
