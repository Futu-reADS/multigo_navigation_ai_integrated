# GitHub Actions & Claude Code Integration - Complete Guide

> **⚡ IMPORTANT UPDATE (2025-11-18):**
> - **No token creation required!** Uses GitHub's built-in `GITHUB_TOKEN`
> - **No secrets to configure!** Everything is automatic
> - **Simplified setup!** Only 3 steps instead of 6
> - See [WORKFLOW_FIX_NOTES.md](./WORKFLOW_FIX_NOTES.md) for details on what changed

## 📑 Table of Contents
- [Introduction](#introduction)
- [What is GitHub Actions?](#what-is-github-actions)
- [What is Claude Code?](#what-is-claude-code)
- [How They Work Together](#how-they-work-together)
- [Complete Setup Guide](#complete-setup-guide)
- [Understanding the Workflow](#understanding-the-workflow)
- [How to Use Claude in Pull Requests](#how-to-use-claude-in-pull-requests)
- [Branch Strategy](#branch-strategy)
- [DevContainer vs GitHub Actions](#devcontainer-vs-github-actions)
- [Troubleshooting](#troubleshooting)
- [FAQs](#faqs)

---

## Introduction

This guide explains the complete setup and usage of GitHub Actions with Claude Code AI integration for the MultiGo Navigation project. It covers everything from basic concepts to advanced usage.

**What you'll learn:**
- What GitHub Actions and Claude Code are
- How they integrate and work together
- Step-by-step setup for private repositories
- How to interact with Claude in your PRs
- Troubleshooting common issues

---

## What is GitHub Actions?

### Overview
**GitHub Actions** is GitHub's built-in CI/CD (Continuous Integration/Continuous Deployment) automation platform.

### Key Concepts

**Workflow**: Automated process defined in YAML files
- Location: `.github/workflows/` directory
- Runs on GitHub's cloud servers
- Triggers on events (PR, push, comments, etc.)

**Events**: Actions that trigger workflows
- Creating a pull request
- Pushing commits
- Adding comments
- Merging code

**Jobs**: Tasks that run in the workflow
- Run tests
- Build code
- Deploy applications
- **Review code with AI (Claude)**

**Runners**: Servers that execute workflows
- GitHub-hosted (ubuntu-latest, windows, macos)
- Self-hosted (your own servers)

### Why Use GitHub Actions?

✅ **Automation**: No manual testing or deployment
✅ **Consistency**: Same process every time
✅ **Quality Control**: Catch bugs before merging
✅ **Collaboration**: Team sees results in PRs
✅ **Free**: Generous free tier for public and private repos

---

## What is Claude Code?

### Overview
**Claude Code** is an AI-powered code review assistant built by Anthropic. It's like having an expert developer review every PR.

### What Claude Does

#### 1. Automated Code Review
```python
# Your code:
def navigate(distance, speed):
    time = distance / speed
    return time

# Claude's review:
⚠️ Division by zero risk detected!

Suggested fix:
def navigate(distance, speed):
    if speed <= 0:
        raise ValueError("Speed must be positive")
    time = distance / speed
    return time
```

#### 2. Interactive Q&A
```
You: @claude How does the SLAM algorithm work here?

Claude: This SLAM implementation uses RTABMap with:
1. Visual odometry from camera data
2. Loop closure detection for drift correction
3. Graph optimization for trajectory refinement
The queue_size=10 ensures reliable data processing.
```

#### 3. Best Practices
```yaml
# Your launch file:
use_sim_time: true

# Claude suggests:
💡 Make this configurable:
DeclareLaunchArgument(
    'use_sim_time',
    default_value='false',
    description='Use simulation time'
)
```

#### 4. Security Analysis
```python
# Your code:
api_key = "sk-1234567890abcdef"

# Claude warns:
🔒 Security Alert: Hardcoded API key detected!
Use environment variables:
api_key = os.getenv('API_KEY')
```

#### 5. Bug Detection
```python
# Your code:
self.publisher = self.create_publisher(Twist, 'cmd_vel')
self.publisher.publish(twist_msg)

# Claude catches:
🐛 Missing queue_size parameter!
self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
```

---

## How They Work Together

### The Complete Flow

```
┌─────────────────────────────────────────────────────────────────┐
│ STEP 1: Developer Action                                        │
│ ┌──────────────────────────────────────────────────────────┐   │
│ │ Developer creates/updates a Pull Request                  │   │
│ │ - Adds new features                                       │   │
│ │ - Fixes bugs                                              │   │
│ │ - Updates documentation                                   │   │
│ └──────────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────────┘
                              ↓
┌─────────────────────────────────────────────────────────────────┐
│ STEP 2: GitHub Detects Event                                    │
│ ┌──────────────────────────────────────────────────────────┐   │
│ │ GitHub triggers: pull_request event                       │   │
│ │ - Event type: opened/synchronized/reopened                │   │
│ │ - Target branch: main/develop/feature/*                   │   │
│ └──────────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────────┘
                              ↓
┌─────────────────────────────────────────────────────────────────┐
│ STEP 3: GitHub Actions Starts                                   │
│ ┌──────────────────────────────────────────────────────────┐   │
│ │ Reads: .github/workflows/claude-code.yml                  │   │
│ │ Starts job on: ubuntu-latest runner                       │   │
│ │ Status: "Claude Code Integration" running...              │   │
│ └──────────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────────┘
                              ↓
┌─────────────────────────────────────────────────────────────────┐
│ STEP 4: Code Checkout                                           │
│ ┌──────────────────────────────────────────────────────────┐   │
│ │ Action: actions/checkout@v4                               │   │
│ │ Downloads your repository code to the runner              │   │
│ │ Includes: All files, commit history, PR diff             │   │
│ └──────────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────────┘
                              ↓
┌─────────────────────────────────────────────────────────────────┐
│ STEP 5: Claude Code Action Executes                             │
│ ┌──────────────────────────────────────────────────────────┐   │
│ │ Action: anthropics/claude-code-action@v1                  │   │
│ │ Authenticates with: Built-in GITHUB_TOKEN                 │   │
│ │ Sends to Claude API:                                      │   │
│ │   - Code diff (what changed)                              │   │
│ │   - File context                                          │   │
│ │   - Commit messages                                       │   │
│ │   - PR description                                        │   │
│ └──────────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────────┘
                              ↓
┌─────────────────────────────────────────────────────────────────┐
│ STEP 6: Claude AI Analysis                                      │
│ ┌──────────────────────────────────────────────────────────┐   │
│ │ Claude analyzes:                                          │   │
│ │ ✓ Code quality and structure                              │   │
│ │ ✓ Potential bugs and errors                               │   │
│ │ ✓ Security vulnerabilities                                │   │
│ │ ✓ Performance issues                                      │   │
│ │ ✓ Best practices compliance                               │   │
│ │ ✓ ROS2-specific patterns                                  │   │
│ └──────────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────────┘
                              ↓
┌─────────────────────────────────────────────────────────────────┐
│ STEP 7: Claude Posts Review                                     │
│ ┌──────────────────────────────────────────────────────────┐   │
│ │ Claude adds comments to PR:                               │   │
│ │ - General review summary                                  │   │
│ │ - Line-specific comments                                  │   │
│ │ - Suggestions and improvements                            │   │
│ │ - Code examples                                           │   │
│ └──────────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────────┘
                              ↓
┌─────────────────────────────────────────────────────────────────┐
│ STEP 8: Interactive Conversation                                │
│ ┌──────────────────────────────────────────────────────────┐   │
│ │ Developer can:                                            │   │
│ │ - Reply to Claude's comments                              │   │
│ │ - Ask follow-up questions                                 │   │
│ │ - Request clarifications                                  │   │
│ │ - Ask for alternative solutions                           │   │
│ │                                                           │   │
│ │ Claude responds to each question!                         │   │
│ └──────────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────────┘
```

### Real Example: Navigation Stack PR

**Scenario**: You update RTABMap configuration

```yaml
# Your changes in config/rtabmap.yaml
rtabmap:
  subscribe_depth: true
  frame_id: base_link
```

**What happens:**

1. **You create PR** → "Update RTABMap depth subscription"

2. **GitHub Actions triggers** → Workflow starts running

3. **Claude analyzes** → Reviews your YAML changes

4. **Claude comments**:
   ```
   ✅ Good: Enabling depth subscription for 3D mapping

   💡 Suggestions:
   1. Add queue_size for reliability:
      queue_size: 10

   2. Consider adding these parameters:
      odom_frame_id: odom
      publish_tf: true

   3. For indoor navigation, enable loop closure:
      Mem/IncrementalMemory: true
      Mem/InitWMWithAllNodes: false
   ```

5. **You reply**:
   ```
   @claude Why do we need queue_size here?
   ```

6. **Claude explains**:
   ```
   Queue size in ROS2 determines the message buffer depth:

   - Without it: Messages may be dropped during processing spikes
   - With queue_size=10: System buffers up to 10 messages
   - For sensor data like depth images: Prevents data loss
   - Recommended: 10 for most cases, 30+ for high-frequency sensors

   In your RTABMap config, depth images arrive at ~30Hz.
   A queue_size of 10 gives ~0.3s buffer for processing delays.
   ```

---

## Complete Setup Guide

### Prerequisites

✅ GitHub repository (private or public)
✅ Git configured locally
✅ Workflow file already exists (already done!)

### ⚡ Quick Setup (No Token Required!)

**Good news!** The Claude Code integration uses GitHub's built-in `GITHUB_TOKEN`, which means:
- ✅ **No manual token creation needed**
- ✅ **No secrets to configure**
- ✅ **Automatically secure**
- ✅ **Zero configuration required**

The workflow is already set up and ready to use!

### Step 1: Verify Workflow File

The workflow file is already created at:
```
.github/workflows/claude-code.yml
```

#### Verify it exists on all branches:

```bash
# Check main branch
git checkout main
ls .github/workflows/claude-code.yml

# Check develop branch
git checkout develop
ls .github/workflows/claude-code.yml

# Check your feature branch
git checkout feature/localization
ls .github/workflows/claude-code.yml
```

All should show the file exists.

### Step 2: Verify GitHub Actions

1. **Go to Actions Tab**
   ```
   https://github.com/Futu-reADS/multigo_navigation/actions
   ```

2. **Check Workflow List**
   - You should see: **"Claude Code Integration"** in the left sidebar
   - Status may show: "No runs yet" or list of recent runs

3. **If you don't see it:**
   - Workflow file must be on `main` branch to appear in the list
   - Check for YAML syntax errors

### Step 3: Test the Integration

#### Option A: Create a Test PR

1. **Make a small change**
   ```bash
   git checkout feature/localization
   echo "# Test Claude Integration" >> README.md
   git add README.md
   git commit -m "Test: Verify Claude Code integration"
   git push origin feature/localization
   ```

2. **Create Pull Request**
   - Go to: https://github.com/Futu-reADS/multigo_navigation
   - Click **"Pull requests"** → **"New pull request"**
   - Base: `develop` ← Compare: `feature/localization`
   - Click **"Create pull request"**

3. **Watch for Claude**
   - Go to **"Checks"** tab in the PR
   - You should see "Claude Code Integration" running
   - Claude will post comments within 2-3 minutes

#### Option B: Comment on Existing PR

If you already have a PR:
```
@claude Please review this pull request
```

Claude will respond!

### Step 4: Confirmation Checklist

✅ Workflow file exists on main, develop, and feature branches
✅ "Claude Code Integration" appears in Actions tab
✅ Test PR created and Claude commented

**If all checked: Setup complete! 🎉**

**Note:** No token creation or secrets needed - GitHub's built-in `GITHUB_TOKEN` is used automatically!

---

## Understanding the Workflow

### Workflow File Location
```
.github/workflows/claude-code.yml
```

### Complete Workflow Explained

```yaml
name: Claude Code Integration
# This name appears in GitHub Actions UI
```

#### Triggers (When workflow runs)

```yaml
on:
  pull_request:
    types: [opened, synchronize, reopened]
    branches:
      - main
      - develop
      - 'feature/**'
```

**Explanation:**
- `pull_request`: Runs when PRs are created or updated
- `opened`: New PR created
- `synchronize`: New commits pushed to existing PR
- `reopened`: Closed PR reopened
- `branches`: Only for PRs targeting these branches

```yaml
  pull_request_target:
    types: [opened, synchronize, reopened]
    branches:
      - main
      - develop
      - 'feature/**'
```

**Explanation:**
- Same as `pull_request` but for PRs from forks
- Allows external contributors to get Claude reviews
- Uses base branch's workflow (more secure)

```yaml
  issue_comment:
    types: [created]
```

**Explanation:**
- Runs when someone comments on PR or issue
- Allows you to ask Claude questions: `@claude explain this`

**Note:** The `push` event is **NOT supported** by Claude Code action and has been removed from the configuration.

#### Permissions

```yaml
permissions:
  contents: write
  pull-requests: write
  issues: write
  checks: write
  statuses: write
```

**Explanation:**

| Permission | What It Allows |
|------------|----------------|
| `contents: write` | Read repository code and files |
| `pull-requests: write` | Post review comments on PRs |
| `issues: write` | Comment on issues |
| `checks: write` | Create check runs (✓ or ✗ in PR) |
| `statuses: write` | Update commit status |

#### Job Definition

```yaml
jobs:
  claude-code:
    runs-on: ubuntu-latest
```

**Explanation:**
- `claude-code`: Job name (appears in logs)
- `runs-on: ubuntu-latest`: Uses GitHub's Ubuntu server

#### Steps

```yaml
    steps:
      - name: Checkout code
        uses: actions/checkout@v4
```

**Explanation:**
- Downloads your repository code to the runner
- Required so Claude can analyze the files

```yaml
      - name: Claude Code Action
        uses: anthropics/claude-code-action@v1
        with:
          github_token: ${{ secrets.GITHUB_TOKEN }}
```

**Explanation:**
- Runs the Claude Code action
- `uses: anthropics/claude-code-action@v1`: Official Claude action
- `github_token`: Parameter name (note: underscore, not hyphen)
- `${{ secrets.GITHUB_TOKEN }}`: GitHub's built-in token (automatically provided)

### Workflow Execution Timeline

```
Time     Event
────────────────────────────────────────────────────
00:00    PR created / Comment added
00:01    GitHub detects event
00:02    Workflow starts
00:05    Code checked out
00:10    Claude action starts
00:15    Code sent to Claude API
00:30    Claude analyzes code
01:00    Claude posts review comments
01:05    Workflow completes
────────────────────────────────────────────────────
         ✓ Total time: ~1-2 minutes
```

---

## How to Use Claude in Pull Requests

### Method 1: Automatic Review

**What happens:** Claude automatically reviews every PR

**You do:**
1. Create PR
2. Wait 1-2 minutes
3. Check PR for Claude's comments

**Claude does:**
- Reviews all changed files
- Posts summary comment
- Adds line-specific comments
- Suggests improvements

**Example:**
```
You create PR: "Add obstacle avoidance to navigation"

Claude automatically comments:

📝 Code Review Summary

✅ Good changes:
- Added dynamic obstacle detection
- Integrated with local costmap

⚠️ Suggestions:
1. Add error handling for sensor failures (line 45)
2. Consider reducing max_velocity for safety (line 78)
3. Add unit tests for collision checking

💡 Recommendations:
- Document the obstacle detection algorithm
- Add parameter validation
```

### Method 2: Ask Questions

**What happens:** You ask Claude specific questions

**Syntax:**
```
@claude [your question]
```

**Examples:**

#### General Question
```
@claude Can you explain how the SLAM algorithm works in this PR?
```

#### Specific Code Question
```
@claude Is the TF transform handling correct on line 145?
```

#### Request Review
```
@claude Please review the navigation parameter changes
```

#### Ask for Alternatives
```
@claude What's a better way to implement this?
```

#### Request Examples
```
@claude Can you show me an example of proper ROS2 lifecycle node setup?
```

### Method 3: Respond to Claude's Comments

**What happens:** Continue conversation with Claude

**Example conversation:**

```
Claude: ⚠️ This function may have a race condition

You: @claude Can you suggest a fix?

Claude: Use a mutex lock:
```
```python
from threading import Lock

class Navigator:
    def __init__(self):
        self.lock = Lock()

    def update_position(self, pos):
        with self.lock:
            self.current_position = pos
```
```

You: @claude Will this impact performance?

Claude: Minimal impact. Mutex locks are fast (~microseconds).
For navigation updates at 10Hz, overhead is negligible.
Only use lock-free alternatives if updating >1kHz.
```

### Method 4: Request Specific Reviews

**What happens:** Ask Claude to check specific aspects

**Examples:**

#### Security Review
```
@claude Check this code for security vulnerabilities
```

#### Performance Review
```
@claude Are there any performance issues in this implementation?
```

#### ROS2 Best Practices
```
@claude Does this follow ROS2 best practices?
```

#### Test Coverage
```
@claude What tests should I add for this code?
```

### Method 5: Get Documentation Help

**What happens:** Claude helps with documentation

**Examples:**

```
@claude Write docstrings for the new functions

@claude Explain this code change for the README

@claude What should I document about this feature?
```

### Tips for Best Results

✅ **Be specific**: "Review the navigation logic" is better than "Review this"
✅ **Provide context**: Explain what you're trying to achieve
✅ **Ask follow-ups**: Claude remembers the conversation
✅ **Reference line numbers**: "Check line 145" helps Claude focus
✅ **Be patient**: Responses take 30-60 seconds

❌ **Avoid vague questions**: "Is this good?" won't get useful answers
❌ **Don't ask multiple unrelated questions**: One topic per comment
❌ **Don't expect Claude to run code**: It analyzes, doesn't execute

---

## Branch Strategy

### Supported Workflow

The integration works with this branching model:

```
main (production)
  ↑
  │ (PR)
  │
develop (integration)
  ↑
  │ (PR)
  │
feature/localization (your work)
feature/navigation
feature/slam
```

### How It Works

#### Scenario 1: Feature → Develop
```bash
# You're working on feature branch
git checkout feature/localization
git commit -m "Add new SLAM feature"
git push origin feature/localization

# Create PR: feature/localization → develop
# ✅ Claude reviews automatically
```

#### Scenario 2: Develop → Main
```bash
# Develop is ready for release
# Create PR: develop → main
# ✅ Claude reviews all changes since last release
```

#### Scenario 3: Feature → Main (Hotfix)
```bash
# Emergency fix
git checkout feature/hotfix-navigation
git commit -m "Fix critical navigation bug"
git push origin feature/hotfix-navigation

# Create PR: feature/hotfix-navigation → main
# ✅ Claude reviews the fix
```

#### Scenario 4: Direct Push
```bash
# Small change directly to develop
git checkout develop
git commit -m "Update README"
git push origin develop

# ✅ Claude analyzes the commit
```

### Configuration for All Branches

The workflow triggers on:
- ✅ `main` branch
- ✅ `develop` branch
- ✅ Any `feature/*` branch (feature/navigation, feature/slam, etc.)

**Pattern matching:**
```yaml
'feature/**'  # Matches:
              # - feature/navigation
              # - feature/slam
              # - feature/localization
              # - feature/path/planning (nested too!)
```

### Best Practices

1. **Always use PRs**: Don't push directly to main
2. **Small PRs**: Easier for Claude (and humans) to review
3. **Descriptive titles**: Helps Claude understand context
4. **One feature per PR**: Focused reviews are better
5. **Respond to Claude**: Engage with suggestions

---

## DevContainer vs GitHub Actions

### Common Confusion

Many developers confuse these two tools. Let's clarify:

### DevContainer

**What it is:**
- Local development environment
- Runs on your computer
- Uses Docker containers
- Provides consistent dev setup

**Purpose:**
- Standardize development environment
- Quick onboarding for new developers
- Isolate project dependencies
- "Works on my machine" solution

**When it runs:**
- When you open project in VS Code
- On your local computer
- While you're coding

**Created by:**
- You (the developer)
- Lives in: `.devcontainer/` folder

**Example DevContainer setup:**
```
.devcontainer/
├── devcontainer.json    # Configuration
└── Dockerfile          # Environment definition
```

**Benefits for ROS2:**
```dockerfile
# Everything pre-installed:
- Ubuntu 22.04
- ROS2 Humble
- Gazebo
- NAV2
- All dependencies
- VS Code extensions
```

**Usage:**
1. Open VS Code
2. Click "Reopen in Container"
3. Everything ready to code!

### GitHub Actions

**What it is:**
- Cloud-based automation
- Runs on GitHub's servers
- Continuous Integration/Deployment
- Automated workflows

**Purpose:**
- Automate testing and deployment
- Code review (with Claude)
- Build and release
- CI/CD pipeline

**When it runs:**
- When you create PR
- When you push commits
- When you add comments
- On schedule (if configured)

**Created by:**
- You (the developer)
- Lives in: `.github/workflows/` folder

**Example:**
```yaml
# .github/workflows/claude-code.yml
# Runs Claude review on every PR
```

### Side-by-Side Comparison

| Feature | DevContainer | GitHub Actions |
|---------|--------------|----------------|
| **Location** | Your computer | GitHub cloud |
| **Purpose** | Development | Automation |
| **When** | While coding | On events (PR, push) |
| **Speed** | Instant | 1-2 minutes |
| **Cost** | Free (uses your CPU) | Free tier available |
| **Internet** | Not required* | Required |
| **Setup** | `.devcontainer/` | `.github/workflows/` |
| **Use case** | Write code | Test/Review code |

*Initial setup requires internet for Docker image

### Do You Need Both?

**DevContainer**: Optional but recommended
- Especially useful for complex setups like ROS2
- Makes team onboarding instant
- Not required for Claude integration

**GitHub Actions**: Required for Claude
- Necessary for automated reviews
- Essential for CI/CD
- Runs independently of DevContainer

### Real-World Usage

**Without DevContainer (current setup):**
```bash
# Developer must manually install:
sudo apt install ros-humble-*
sudo apt install gazebo
sudo apt install nav2
# ... 20 more commands ...

# Takes: 1-2 hours
# Problems: Version mismatches, missing dependencies
```

**With DevContainer:**
```bash
# Developer does:
1. Clone repo
2. Open in VS Code
3. Click "Reopen in Container"

# Takes: 5 minutes (first time)
# Problems: None! Everything configured
```

**GitHub Actions (independent):**
```bash
# Runs automatically on PR
# No developer action needed
# Claude reviews and comments
```

### Should You Create a DevContainer?

**Benefits for your ROS2 project:**
- ✅ Complex setup (ROS2, Gazebo, NAV2, RTABMap)
- ✅ Multiple dependencies
- ✅ Team collaboration
- ✅ New developers join quickly
- ✅ Consistent environment

**Recommendation:** YES, create DevContainer

**Would you like me to create one?** It would include:
- Ubuntu 22.04
- ROS2 Humble pre-installed
- Gazebo Simulator
- NAV2 navigation stack
- All your dependencies (rtabmap, pcl-ros, etc.)
- VS Code extensions for ROS2

---

## Troubleshooting

### Issue 1: Workflow Not Visible in Actions

**Symptom:**
Can't see "Claude Code Integration" at:
```
https://github.com/Futu-reADS/multigo_navigation/actions
```

**Diagnosis:**
```bash
# Check if file exists on main branch
git checkout main
git ls-tree -r HEAD --name-only | grep claude-code.yml
```

**Solutions:**

✅ **Solution 1**: Ensure workflow is on main branch
```bash
git checkout main
git pull origin main
# Verify file exists:
ls .github/workflows/claude-code.yml
```

✅ **Solution 2**: Check for YAML syntax errors
```bash
# Install yamllint
pip install yamllint

# Validate syntax
yamllint .github/workflows/claude-code.yml
```

✅ **Solution 3**: Push workflow to main
```bash
git checkout main
# Ensure workflow file exists
git add .github/workflows/claude-code.yml
git commit -m "Add workflow to main"
git push origin main
```

### Issue 2: Workflow Fails - Authentication Error

**Symptom:**
Workflow runs but shows error:
```
Error: Authentication failed
Status: 401 Unauthorized
```

**Diagnosis:**
Usually caused by workflow configuration issues

**Solutions:**

✅ **Solution 1**: Verify workflow file uses `GITHUB_TOKEN`
Check that `.github/workflows/claude-code.yml` contains:
```yaml
with:
  github_token: ${{ secrets.GITHUB_TOKEN }}
```

**NOT:**
```yaml
with:
  github-token: ${{ secrets.CLAUDE_CODE_TOKEN }}  # Wrong!
```

✅ **Solution 2**: Check GitHub Actions permissions
1. Go to: Settings → Actions → General
2. Under "Workflow permissions"
3. Ensure "Read and write permissions" is selected
4. Check "Allow GitHub Actions to create and approve pull requests"

✅ **Solution 3**: Re-run the workflow
Sometimes a simple re-run fixes temporary auth issues:
1. Go to Actions tab
2. Click failed workflow
3. Click "Re-run jobs"

### Issue 3: Claude Doesn't Respond to Comments

**Symptom:**
You comment `@claude review this` but get no response

**Diagnosis:**
Comment type or workflow trigger issue

**Solutions:**

✅ **Solution 1**: Verify comment location
- ✅ **DO**: Comment in the PR conversation tab
- ❌ **DON'T**: Comment in "Files changed" review comments
- ❌ **DON'T**: Comment in a specific line review

✅ **Solution 2**: Check workflow has issue_comment trigger
```yaml
# Should be in .github/workflows/claude-code.yml
on:
  issue_comment:
    types: [created]
```

✅ **Solution 3**: Wait longer
- Claude can take 1-3 minutes to respond
- Check "Actions" tab for running workflows

✅ **Solution 4**: Check Actions tab for errors
1. Go to: Actions tab
2. Find the workflow run
3. Check logs for errors

### Issue 4: Workflow Not Triggering on PR

**Symptom:**
Created PR but workflow doesn't run

**Diagnosis:**
Branch or trigger configuration issue

**Solutions:**

✅ **Solution 1**: Verify target branch
PR must target: `main`, `develop`, or `feature/*`

```yaml
# Check your workflow file has:
branches:
  - main
  - develop
  - 'feature/**'
```

✅ **Solution 2**: Check if Actions are enabled
1. Go to: Settings → Actions → General
2. Ensure "Allow all actions and reusable workflows" is selected

✅ **Solution 3**: Verify workflow exists on target branch
```bash
# If PR targets develop:
git checkout develop
ls .github/workflows/claude-code.yml  # Should exist
```

✅ **Solution 4**: Check PR event types
```yaml
# Workflow should have:
pull_request:
  types: [opened, synchronize, reopened]
```

### Issue 5: Private Repository Issues

**Symptom:**
Error: "Repository not found" or "403 Forbidden"

**Solutions:**

✅ **Solution 1**: Verify token repository access
1. Go to: https://github.com/settings/tokens
2. Click on your token
3. Under "Repository access", ensure `multigo_navigation` is selected

✅ **Solution 2**: Check organization permissions
1. Go to: Organization settings (if using organization)
2. Actions → General
3. Ensure Actions are enabled

✅ **Solution 3**: Verify you're admin
- You need admin access to install workflows
- Check: Repository → Settings (if you can't see Settings, you're not admin)

### Issue 6: Unsupported Event Type Error

**Symptom:**
```
Error: Prepare step failed with error: Unsupported event type: push
```

**Diagnosis:**
The workflow has a `push` event trigger, which is not supported by Claude Code

**Solutions:**

✅ **Solution 1**: Remove push event from workflow
Edit `.github/workflows/claude-code.yml` and remove:
```yaml
push:
  branches:
    - main
    - develop
    - 'feature/**'
```

✅ **Solution 2**: Use Pull Requests instead
Claude Code only works with PRs. Instead of direct pushes:
1. Create a feature branch
2. Make changes
3. Create a PR
4. Claude reviews the PR

### Issue 7: Workflow Takes Too Long

**Symptom:**
Workflow runs for >5 minutes or times out

**Solutions:**

✅ **Solution 1**: Check GitHub status
- Visit: https://www.githubstatus.com/
- Actions may be slow during outages

✅ **Solution 2**: Check large PR size
- Very large PRs (100+ files) take longer
- Consider splitting into smaller PRs

✅ **Solution 3**: Retry workflow
1. Go to failed workflow run
2. Click "Re-run jobs"

### Getting More Help

If issues persist:

1. **Check workflow logs:**
   - Actions tab → Click workflow run → View logs
   - Look for specific error messages

2. **Verify setup checklist:**
   - [ ] Workflow file uses `github_token` (not `github-token`)
   - [ ] Workflow uses `GITHUB_TOKEN` (not `CLAUDE_CODE_TOKEN`)
   - [ ] Workflow file on main branch
   - [ ] Workflow file syntax valid
   - [ ] No `push` event in triggers
   - [ ] GitHub Actions enabled
   - [ ] PR targets supported branch (main, develop, or feature/*)

3. **Test with minimal PR:**
   - Create tiny PR (1 line change)
   - Isolates setup vs. content issues

4. **Check repository:**
   - Settings → Actions → General
   - Ensure "Allow all actions" enabled

---

## FAQs

### General Questions

**Q: Does this work for private repositories?**
A: Yes! The built-in `GITHUB_TOKEN` automatically has access to your private repo.

**Q: How much does this cost?**
A: GitHub Actions has a generous free tier:
- Public repos: Unlimited
- Private repos: 2,000 minutes/month free
- Claude reviews typically take <1 minute each

**Q: Can I use this with other branches?**
A: Yes! The workflow supports custom branch patterns. Edit:
```yaml
branches:
  - main
  - develop
  - 'your-pattern/*'
```

**Q: Will Claude see all my code?**
A: Claude only sees:
- Files changed in the PR
- Surrounding context for understanding
- Not your entire codebase (unless PR touches it)

**Q: Can I turn off Claude temporarily?**
A: Yes, two options:
1. Disable workflow in Actions settings
2. Delete/rename workflow file temporarily

### Security Questions

**Q: Is my code secure?**
A: Yes:
- Claude API is encrypted (HTTPS)
- Anthropic has strict privacy policies
- Token stored as encrypted secret
- Only PR diffs are sent, not entire repo

**Q: Can I use this for proprietary code?**
A: Yes, but review your company's policies:
- Some orgs prohibit sending code to external APIs
- Check with legal/security team first
- Consider Anthropic's data retention policy

**Q: What if my token leaks?**
A: Immediately:
1. Revoke token: https://github.com/settings/tokens
2. Create new token
3. Update secret
4. Check Actions audit log for unauthorized usage

### Usage Questions

**Q: Can Claude fix bugs automatically?**
A: No, Claude:
- ✅ Suggests fixes
- ✅ Shows example code
- ❌ Doesn't commit changes automatically
- You must apply fixes manually

**Q: How many questions can I ask Claude?**
A: Unlimited within reason:
- GitHub Actions has usage limits
- Each interaction costs ~30-60 seconds of Actions time
- Stay under monthly free tier: 2,000 minutes

**Q: Does Claude learn from my code?**
A: No:
- Claude doesn't train on your specific code
- Each review is independent
- Privacy-focused design

**Q: Can Claude review non-code files?**
A: Yes! Claude reviews:
- ✅ Code (.py, .cpp, .yaml, etc.)
- ✅ Documentation (.md, .rst)
- ✅ Configuration (launch files, yaml)
- ✅ Scripts (.sh, .bash)

**Q: What languages does Claude support?**
A: All major languages:
- Python (excellent for ROS2)
- C++ (excellent for ROS2)
- YAML
- XML (launch files)
- Bash/Shell
- And 50+ others

### Technical Questions

**Q: Can I customize Claude's behavior?**
A: Currently limited:
- Can't change review style
- Can't set custom rules
- Can provide context in PR description

**Q: Does this replace code review?**
A: No! Claude supplements human review:
- Claude: Catches common bugs, patterns
- Humans: Business logic, architecture decisions
- Best together!

**Q: Can I use multiple AI reviewers?**
A: Yes, you can add other GitHub Actions:
- CodeRabbit
- GitHub Copilot
- SonarCloud
- They work in parallel

**Q: What if Claude is wrong?**
A: Claude can make mistakes:
- Always verify suggestions
- Ask Claude to clarify
- Use your judgment
- Report serious issues to Anthropic

### ROS2-Specific Questions

**Q: Does Claude understand ROS2?**
A: Yes! Claude is trained on:
- ROS2 documentation
- Common ROS2 patterns
- Best practices
- Navigation stack specifics

**Q: Can Claude help with launch files?**
A: Yes, Claude reviews:
- Python launch files
- XML launch files
- Parameter YAML files
- Node configurations

**Q: Does Claude know about NAV2?**
A: Yes, Claude understands:
- Navigation2 stack
- Behavior trees
- Controllers and planners
- Configuration parameters

**Q: Can Claude help with Gazebo?**
A: Yes, including:
- World files
- Robot descriptions (URDF/SDF)
- Plugins
- Simulation parameters

---

## Quick Reference

### Essential Links

| Resource | URL |
|----------|-----|
| Create Token | https://github.com/settings/personal-access-tokens/new |
| Repository Secrets | https://github.com/Futu-reADS/multigo_navigation/settings/secrets/actions |
| Actions Dashboard | https://github.com/Futu-reADS/multigo_navigation/actions |
| GitHub Status | https://www.githubstatus.com/ |

### Key Commands

```bash
# Check workflow file exists
ls .github/workflows/claude-code.yml

# Validate YAML syntax
yamllint .github/workflows/claude-code.yml

# Check workflow on remote
git ls-tree -r origin/main --name-only | grep claude

# View recent workflow runs
# (Use GitHub UI - no CLI equivalent)
```

### Claude Interaction Examples

```bash
# Automatic review - just create PR
# No command needed

# Ask question
@claude Can you explain this code?

# Request specific review
@claude Check for security issues

# Ask for examples
@claude Show me a better way to do this

# Follow-up question
@claude Why is that necessary?
```

### Token Permissions Checklist

- [ ] Actions: Read and write
- [ ] Checks: Read and write
- [ ] Contents: Read and write
- [ ] Metadata: Read-only
- [ ] Pull requests: Read and write
- [ ] Workflows: Read and write

### Workflow Trigger Checklist

- [ ] `pull_request` - PRs created/updated
- [ ] `pull_request_target` - PRs from forks
- [ ] `issue_comment` - Questions to Claude
- [ ] `push` - Direct commits

---

## Summary

### What You've Learned

1. **GitHub Actions** = Automation platform running on GitHub's cloud
2. **Claude Code** = AI code reviewer that comments on PRs
3. **Together** = Automated, intelligent code review on every PR
4. **Setup** = Create token → Add secret → Workflow runs automatically
5. **Usage** = Create PR or comment `@claude [question]`
6. **Branch Strategy** = Works on main, develop, and all feature branches
7. **DevContainer** = Different tool (local dev environment), not required

### Key Takeaways

✅ Claude reviews all PRs automatically
✅ You can ask Claude questions anytime
✅ Works on private repositories
✅ Free tier covers most usage
✅ Supports your branch workflow
✅ Complements human code review
✅ Improves code quality and catches bugs

### Next Steps

1. **Complete setup** if not done:
   - Create token
   - Add secret
   - Test with PR

2. **Start using Claude**:
   - Create a PR
   - Wait for review
   - Ask questions

3. **Share with team**:
   - Send them this documentation
   - Show example PR with Claude
   - Encourage adoption

4. **Optional enhancements**:
   - Create DevContainer for easier development
   - Add more GitHub Actions (tests, builds)
   - Customize workflow for your needs

---

## Document Information

**Created**: 2025-11-18
**Version**: 1.0
**Maintained by**: Development Team
**Repository**: https://github.com/Futu-reADS/multigo_navigation

---

**Ready to start?** Create your first PR and watch Claude in action! 🚀
