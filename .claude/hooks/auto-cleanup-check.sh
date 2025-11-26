#!/bin/bash

###############################################################################
# Auto Cleanup Check
#
# Checks if PR is merged and prompts Claude to run cleanup
# Called automatically by Claude after merge operations
###############################################################################

BRANCH=$(git branch --show-current 2>/dev/null || echo "")
PR_NUM=$(gh pr list --head "$BRANCH" --json number --jq '.[0].number' 2>/dev/null || echo "")

if [ -z "$PR_NUM" ]; then
    # No PR for this branch
    exit 0
fi

# Check if PR is merged
PR_STATE=$(gh pr view "$PR_NUM" --json state --jq '.state' 2>/dev/null || echo "")

if [ "$PR_STATE" = "MERGED" ]; then
    echo ""
    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    echo "✓ PR #$PR_NUM is merged!"
    echo ""
    echo "Cleanup recommendation:"
    echo "  Run: ./scripts/cleanup-after-merge.sh"
    echo ""
    echo "This will:"
    echo "  • Switch to main branch"
    echo "  • Pull merged changes"
    echo "  • Delete feature branch"
    echo "  • Ready for next issue"
    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    echo ""
fi
