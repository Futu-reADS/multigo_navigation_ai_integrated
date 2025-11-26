#!/bin/bash

###############################################################################
# Automation System Verification Script
#
# This script verifies that the workflow automation system is properly set up
# and all dependencies are available.
#
# Usage:
#   ./scripts/verify-automation.sh
###############################################################################

# Note: Not using 'set -e' to allow script to continue and report all issues

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

ERRORS=0
WARNINGS=0

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
    ((ERRORS++))
}

print_warning() {
    echo -e "${YELLOW}⚠${NC} $1"
    ((WARNINGS++))
}

print_info() {
    echo -e "${YELLOW}ℹ${NC} $1"
}

###############################################################################
# Check Dependencies
###############################################################################

print_header "Checking Dependencies"

# Check GitHub CLI
if command -v gh &> /dev/null; then
    GH_VERSION=$(gh --version | head -1)
    print_success "GitHub CLI installed: $GH_VERSION"

    # Check authentication
    if gh auth status &> /dev/null; then
        print_success "GitHub authentication verified"
    else
        print_error "GitHub CLI not authenticated (run: gh auth login)"
    fi
else
    print_error "GitHub CLI (gh) not installed"
    print_info "Install from: https://cli.github.com/"
fi

# Check jq
if command -v jq &> /dev/null; then
    JQ_VERSION=$(jq --version)
    print_success "jq installed: $JQ_VERSION"
else
    print_error "jq not installed (sudo apt install jq)"
fi

# Check node
if command -v node &> /dev/null; then
    NODE_VERSION=$(node --version)
    print_success "Node.js installed: $NODE_VERSION"
else
    print_warning "Node.js not installed (optional for some hooks)"
    print_info "Install from: https://nodejs.org/"
fi

# Check git
if command -v git &> /dev/null; then
    GIT_VERSION=$(git --version)
    print_success "Git installed: $GIT_VERSION"
else
    print_error "Git not installed"
fi

# Check if in git repository
if git rev-parse --git-dir > /dev/null 2>&1; then
    print_success "Running in git repository"
else
    print_error "Not in a git repository"
fi

###############################################################################
# Check File Structure
###############################################################################

echo ""
print_header "Checking File Structure"

# Check automation scripts
SCRIPTS=(
    "scripts/start-work.sh"
    "scripts/cleanup-after-merge.sh"
    ".claude/hooks/post-summary.sh"
)

for script in "${SCRIPTS[@]}"; do
    if [ -f "$script" ]; then
        if [ -x "$script" ]; then
            print_success "$script (executable)"
        else
            print_warning "$script (not executable - run: chmod +x $script)"
        fi
    else
        print_error "$script (missing)"
    fi
done

# Check documentation
DOCS=(
    "CLAUDE.md"
    ".claude/WORKFLOW.md"
    "AUTOMATION-FAQ.md"
    "HOOKS-SETUP.md"
)

for doc in "${DOCS[@]}"; do
    if [ -f "$doc" ]; then
        print_success "$doc"
    else
        print_warning "$doc (missing)"
    fi
done

# Check configuration
if [ -f ".claude/settings.json" ]; then
    print_success ".claude/settings.json (hooks active)"
elif [ -f ".claude/settings.json.template" ]; then
    print_warning ".claude/settings.json (not active - copy from template)"
else
    print_error ".claude/settings.json.template (missing)"
fi

# Check .gitignore
if [ -f ".gitignore" ]; then
    if grep -q ".claude/session-counter.json" .gitignore 2>/dev/null; then
        HAS_ART=0
    else
        HAS_ART=1
    fi

    if [ $HAS_ART -eq 0 ]; then
        print_success ".gitignore (automation artifacts excluded)"
    else
        print_warning ".gitignore (missing automation artifact entries)"
        echo -e "${YELLOW}ℹ${NC} Adding automation artifacts to .gitignore..."
        cat >> .gitignore << 'EOF'

# Claude Code automation artifacts
.claude/session-counter.json
.claude/session-tracking.json
.claude-prompt-*.md
docs/dev-logs/
EOF
        print_success "Added automation artifacts to .gitignore"
    fi
else
    print_warning ".gitignore (missing - creating)"
    cat > .gitignore << 'EOF'
# Claude Code automation artifacts
.claude/session-counter.json
.claude/session-tracking.json
.claude-prompt-*.md
docs/dev-logs/
EOF
    print_success "Created .gitignore with automation artifacts"
fi

###############################################################################
# Check ROS 2 Project Structure
###############################################################################

echo ""
print_header "Checking ROS 2 Project Structure"

# Check if src/ directory exists
if [ -d "src" ]; then
    print_success "src/ directory found"

    # Check key ROS 2 navigation packages
    ROS2_PACKAGES=("nav_control" "nav_docking" "nav_goal" "aruco_detect" "camera_publisher")
    FOUND_PACKAGES=0

    for pkg in "${ROS2_PACKAGES[@]}"; do
        if [ -d "src/$pkg" ]; then
            ((FOUND_PACKAGES++))
        fi
    done

    if [ $FOUND_PACKAGES -eq ${#ROS2_PACKAGES[@]} ]; then
        print_success "All key ROS 2 packages found ($FOUND_PACKAGES/${#ROS2_PACKAGES[@]})"
    elif [ $FOUND_PACKAGES -gt 0 ]; then
        print_warning "Some ROS 2 packages found ($FOUND_PACKAGES/${#ROS2_PACKAGES[@]})"
    else
        print_warning "No key ROS 2 packages found in src/"
    fi
else
    print_warning "src/ directory not found (not in ROS 2 workspace root?)"
fi

###############################################################################
# Check GitHub Integration
###############################################################################

echo ""
print_header "Checking GitHub Integration"

# Check workflows
WORKFLOWS=(
    ".github/workflows/claude.yml"
    ".github/workflows/claude-code-review.yml"
)

for workflow in "${WORKFLOWS[@]}"; do
    if [ -f "$workflow" ]; then
        print_success "$workflow"
    else
        print_warning "$workflow (missing)"
    fi
done

# Check templates
if [ -d ".github/ISSUE_TEMPLATE" ]; then
    TEMPLATE_COUNT=$(find .github/ISSUE_TEMPLATE -name "*.md" | wc -l)
    print_success ".github/ISSUE_TEMPLATE ($TEMPLATE_COUNT templates)"
else
    print_warning ".github/ISSUE_TEMPLATE (missing)"
fi

if [ -f ".github/PULL_REQUEST_TEMPLATE.md" ]; then
    print_success ".github/PULL_REQUEST_TEMPLATE.md"
else
    print_warning ".github/PULL_REQUEST_TEMPLATE.md (missing)"
fi

###############################################################################
# Summary
###############################################################################

echo ""
print_header "Verification Summary"

if [ $ERRORS -eq 0 ] && [ $WARNINGS -eq 0 ]; then
    print_success "All checks passed! Automation system is ready."
    exit 0
elif [ $ERRORS -eq 0 ]; then
    print_warning "$WARNINGS warning(s) - System is functional but some optional components are missing"
    exit 0
else
    print_error "$ERRORS error(s), $WARNINGS warning(s)"
    echo ""
    echo -e "${RED}❌ Automation system has critical issues that need to be fixed${NC}"
    exit 1
fi
