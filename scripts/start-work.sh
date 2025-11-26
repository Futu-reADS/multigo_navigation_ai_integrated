#!/bin/bash

# ==============================================================================
# Workflow Helper: Start Working on GitHub Issue
# ==============================================================================
#
# This script automates the workflow:
# 1. Fetches issue details from GitHub
# 2. Creates appropriate branch
# 3. Prepares Claude Code prompt with context
# 4. Sets up development environment
#
# Usage:
#   ./scripts/start-work.sh <issue-number>
#
# Example:
#   ./scripts/start-work.sh 45
#
# Requirements:
#   - gh (GitHub CLI) installed
#   - jq for JSON parsing
# ==============================================================================

set -euo pipefail  # Fail on errors, undefined vars, pipe failures
IFS=$'\n\t'        # Prevent word splitting issues

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# ==============================================================================
# FUNCTIONS
# ==============================================================================

print_header() {
    echo -e "${BLUE}════════════════════════════════════════${NC}"
    echo -e "${BLUE}  $1${NC}"
    echo -e "${BLUE}════════════════════════════════════════${NC}"
}

print_success() {
    echo -e "${GREEN}✓${NC} $1"
}

print_error() {
    echo -e "${RED}✗${NC} $1"
}

print_info() {
    echo -e "${YELLOW}ℹ${NC} $1"
}

check_requirements() {
    print_header "Checking Requirements"

    if ! command -v gh &> /dev/null; then
        print_error "GitHub CLI (gh) not installed"
        echo "Install: https://cli.github.com/"
        exit 1
    fi
    print_success "GitHub CLI found"

    if ! command -v jq &> /dev/null; then
        print_error "jq not installed"
        echo "Install: sudo apt install jq"
        exit 1
    fi
    print_success "jq found"

    if ! gh auth status &> /dev/null; then
        print_error "Not authenticated with GitHub"
        echo "Run: gh auth login"
        exit 1
    fi
    print_success "GitHub authentication verified"
}

fetch_issue() {
    local issue_num=$1

    print_header "Fetching Issue #${issue_num}"

    # Fetch issue details
    issue_json=$(gh issue view "$issue_num" --json title,body,labels,assignees)

    if [ $? -ne 0 ]; then
        print_error "Failed to fetch issue #${issue_num}"
        exit 1
    fi

    # Parse issue data
    issue_title=$(echo "$issue_json" | jq -r '.title')
    issue_body=$(echo "$issue_json" | jq -r '.body')
    issue_labels=$(echo "$issue_json" | jq -r '.labels[].name' | tr '\n' ',' | sed 's/,$//')

    print_success "Issue fetched: ${issue_title}"
    print_info "Labels: ${issue_labels}"

    # Determine issue type from labels
    if echo "$issue_labels" | grep -qi "bug"; then
        issue_type="fix"
    else
        issue_type="feature"
    fi

    print_info "Issue type: ${issue_type}"
}

create_branch() {
    local issue_num=$1

    print_header "Creating Branch"

    # Generate branch name from issue title
    branch_suffix=$(echo "$issue_title" | tr '[:upper:]' '[:lower:]' | sed 's/[^a-z0-9]/-/g' | sed 's/--*/-/g' | cut -c1-30)
    branch_name="${issue_type}/${issue_num}-${branch_suffix}"

    # Check if branch already exists
    if git show-ref --quiet refs/heads/"$branch_name"; then
        print_info "Branch already exists: ${branch_name}"
        git checkout "$branch_name"
    else
        git checkout -b "$branch_name"
        print_success "Created and switched to branch: ${branch_name}"
    fi
}

extract_claude_context() {
    print_header "Extracting Context from Issue"

    # Extract sections from issue body
    architecture=$(echo "$issue_body" | sed -n '/## 🏗️ Architecture Discussion/,/## 📁 Affected Files/p' | sed '1d;$d')
    affected_files=$(echo "$issue_body" | sed -n '/## 📁 Affected Files/,/## 🔗 Related Issues/p' | sed '1d;$d')
    acceptance_criteria=$(echo "$issue_body" | sed -n '/## 🎯 Acceptance Criteria/,/## 🏗️ Architecture/p' | sed '1d;$d')

    # Count @claude responses in issue comments
    claude_responses=$(gh issue view "$1" --comments --json comments | jq -r '.comments[] | select(.author.login == "claude") | .body' 2>/dev/null || echo "")

    if [ -n "$claude_responses" ]; then
        print_success "Found @claude discussion in issue"
    else
        print_info "No @claude discussion found (you may want to add context)"
    fi
}

generate_prompt() {
    local issue_num=$1

    print_header "Generating Claude Code Prompt"

    # Create prompt file
    prompt_file=".claude-prompt-issue-${issue_num}.md"

    cat > "$prompt_file" << EOF
[WORKFLOW-SCRIPT] Claude Code Prompt - Issue #${issue_num}

## Context from GitHub Issue

**Issue:** #${issue_num} - ${issue_title}
**Type:** ${issue_type}
**Branch:** ${branch_name}

---

## Acceptance Criteria
${acceptance_criteria}

---

## Architecture/Solution (from @claude discussion)
${architecture}

${claude_responses:+
## @claude Recommendations
${claude_responses}
}

---

## Files to Modify
${affected_files}

---

## Your Task

Please implement this ${issue_type} according to the specifications above.

**Requirements:**
1. Follow the architecture discussed in the issue
2. Implement all acceptance criteria
3. Write/update tests
4. Ensure existing tests pass
5. Commit with proper message: "${issue_type}: <description> (#${issue_num})"

**Note:** This session will be auto-logged to docs/dev-logs/issue-${issue_num}.md
EOF

    print_success "Prompt file created: ${prompt_file}"
    print_info "You can copy this prompt or reference it in Claude Code"

    # Display the prompt
    echo ""
    echo -e "${YELLOW}═══════════════════════════════════════════════════════${NC}"
    echo -e "${YELLOW}Suggested Claude Code Prompt:${NC}"
    echo -e "${YELLOW}═══════════════════════════════════════════════════════${NC}"
    cat "$prompt_file"
    echo -e "${YELLOW}═══════════════════════════════════════════════════════${NC}"
}

setup_environment() {
    print_header "Setting Up Environment"

    # Ensure dev-logs directory exists
    mkdir -p docs/dev-logs

    # Create log file for this issue if it doesn't exist
    log_file="docs/dev-logs/issue-${1}.md"
    if [ ! -f "$log_file" ]; then
        cat > "$log_file" << EOF
# Development Log - Issue #${1}

**Issue:** ${issue_title}
**Type:** ${issue_type}
**Branch:** ${branch_name}
**Started:** $(date '+%Y-%m-%d %H:%M:%S')

---

_This file is auto-updated by Claude Code prompt logger hook_

---

EOF
        print_success "Created log file: ${log_file}"
    else
        print_info "Log file already exists: ${log_file}"
    fi

    print_success "Environment ready"
}

display_next_steps() {
    print_header "Ready to Code!"

    echo ""
    echo -e "${GREEN}✓ All set! Next steps:${NC}"
    echo ""
    echo "1. Start Claude Code in this directory"
    echo "2. Copy the prompt above (or from: .claude-prompt-issue-${1}.md)"
    echo "3. Start implementing!"
    echo ""
    echo -e "${BLUE}Your changes will be auto-logged to: docs/dev-logs/issue-${1}.md${NC}"
    echo ""
    echo -e "${YELLOW}When done:${NC}"
    echo "  - Review changes: git diff"
    echo "  - Create PR: gh pr create --fill"
    echo "  - PR merge will auto-close issue #${1}"
    echo ""
}

# ==============================================================================
# MAIN
# ==============================================================================

main() {
    # Check if issue number provided
    if [ $# -eq 0 ]; then
        print_error "Usage: $0 <issue-number>"
        echo ""
        echo "Example:"
        echo "  $0 45"
        exit 1
    fi

    issue_num=$1

    # Security: Validate issue number is numeric (prevent path traversal)
    if ! [[ "$issue_num" =~ ^[0-9]+$ ]]; then
        print_error "Invalid issue number: '$issue_num'"
        echo "Issue number must be a positive integer"
        exit 1
    fi

    # Run workflow
    check_requirements
    fetch_issue "$issue_num"
    create_branch "$issue_num"
    extract_claude_context "$issue_num"
    generate_prompt "$issue_num"
    setup_environment "$issue_num"
    display_next_steps "$issue_num"
}

main "$@"
