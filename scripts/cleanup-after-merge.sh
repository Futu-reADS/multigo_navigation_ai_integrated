#!/bin/bash

###############################################################################
# Cleanup After PR Merge
#
# Automatically cleanup after a PR is merged:
# 1. Switch to main branch
# 2. Pull latest changes (includes merged PR)
# 3. Delete the feature branch locally
# 4. Optionally delete remote branch
#
# Usage:
#   ./scripts/cleanup-after-merge.sh
#   (Run from feature branch after PR is merged on GitHub)
#
# Or specify PR number:
#   ./scripts/cleanup-after-merge.sh 20
###############################################################################

set -e

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

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

# Get current branch
CURRENT_BRANCH=$(git branch --show-current)
BASE_BRANCH="main"

# Extract issue/PR info from branch name
ISSUE_NUM=$(echo "$CURRENT_BRANCH" | grep -oP '(?:feature|fix|issue|refactor|chore)/(\d+)' | grep -oP '\d+' || echo "")

print_header "Post-Merge Cleanup"

# Check if on a feature branch
if [ "$CURRENT_BRANCH" = "$BASE_BRANCH" ]; then
    print_error "Already on $BASE_BRANCH branch"
    echo ""
    print_info "This script should be run from a feature branch after PR is merged"
    echo ""
    echo "Current branches:"
    git branch -a
    exit 1
fi

echo ""
print_info "Current branch: $CURRENT_BRANCH"
print_info "Target branch: $BASE_BRANCH"

# Check if PR number provided or detect from branch
if [ -n "$1" ]; then
    PR_NUM="$1"
else
    PR_NUM=$(gh pr list --head "$CURRENT_BRANCH" --json number --jq '.[0].number' 2>/dev/null || echo "")
fi

# Verify PR is merged (if we have PR number)
if [ -n "$PR_NUM" ]; then
    echo ""
    print_header "Checking PR Status"

    PR_STATE=$(gh pr view "$PR_NUM" --json state --jq '.state' 2>/dev/null || echo "UNKNOWN")

    if [ "$PR_STATE" = "MERGED" ]; then
        print_success "PR #$PR_NUM is merged"
    elif [ "$PR_STATE" = "OPEN" ]; then
        print_error "PR #$PR_NUM is still OPEN (not merged yet)"
        echo ""
        print_info "Please merge the PR on GitHub first, then run this script"
        exit 1
    elif [ "$PR_STATE" = "CLOSED" ]; then
        print_error "PR #$PR_NUM is CLOSED (not merged)"
        echo ""
        print_info "This PR was closed without merging"
        read -p "Continue with cleanup anyway? (y/N) " -n 1 -r
        echo
        if [[ ! $REPLY =~ ^[Yy]$ ]]; then
            exit 1
        fi
    else
        print_info "Could not verify PR status (continuing anyway)"
    fi
fi

# Step 1: Switch to main
echo ""
print_header "Step 1: Switch to $BASE_BRANCH"

git checkout "$BASE_BRANCH"
print_success "Switched to $BASE_BRANCH branch"

# Step 2: Pull latest changes
echo ""
print_header "Step 2: Pull Latest Changes"

git pull origin "$BASE_BRANCH"
print_success "Pulled latest changes from origin/$BASE_BRANCH"

# Step 3: Delete local feature branch
echo ""
print_header "Step 3: Delete Local Feature Branch"

git branch -d "$CURRENT_BRANCH" 2>/dev/null && \
    print_success "Deleted local branch: $CURRENT_BRANCH" || \
    {
        print_info "Branch has unmerged changes, using force delete"
        git branch -D "$CURRENT_BRANCH"
        print_success "Force deleted local branch: $CURRENT_BRANCH"
    }

# Step 4: Check remote branch status
echo ""
print_header "Step 4: Remote Branch Status"

REMOTE_EXISTS=$(git ls-remote --heads origin "$CURRENT_BRANCH" 2>/dev/null)

if [ -n "$REMOTE_EXISTS" ]; then
    print_info "Remote branch origin/$CURRENT_BRANCH still exists"
    print_info "Tip: Check 'Delete branch' when merging PR on GitHub"
    echo ""
    read -p "Delete remote branch now? (y/N) " -n 1 -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        git push origin --delete "$CURRENT_BRANCH"
        print_success "Deleted remote branch: origin/$CURRENT_BRANCH"
    else
        print_info "Skipped - remote branch will remain"
    fi
else
    print_success "Remote branch already deleted (GitHub auto-delete)"
fi

# Final summary
echo ""
print_header "Cleanup Complete!"

echo ""
echo -e "${GREEN}✓ Current branch:${NC} $BASE_BRANCH"
echo -e "${GREEN}✓ Local feature branch:${NC} deleted"
echo -e "${GREEN}✓ Latest changes:${NC} pulled from origin"

if [ -n "$PR_NUM" ] && [ -n "$ISSUE_NUM" ]; then
    echo -e "${GREEN}✓ PR #$PR_NUM merged${NC}"
    echo -e "${GREEN}✓ Issue #$ISSUE_NUM closed${NC}"
fi

echo ""
echo -e "${BLUE}Ready for next issue!${NC}"
echo ""
echo "Run: ./scripts/start-work.sh <issue-number>"
echo ""
