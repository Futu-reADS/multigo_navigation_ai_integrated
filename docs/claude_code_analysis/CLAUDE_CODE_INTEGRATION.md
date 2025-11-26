# Claude Code GitHub Actions Integration

> **⚠️ IMPORTANT:** This document contains outdated setup instructions.
> Please refer to [WORKFLOW_FIX_NOTES.md](./WORKFLOW_FIX_NOTES.md) for the current, correct setup.
>
> **Key changes:**
> - No token creation needed - uses built-in `GITHUB_TOKEN`
> - No secrets to configure
> - Much simpler setup process

## Table of Contents
1. [Overview](#overview)
2. [How Claude Works in Pull Requests](#how-claude-works-in-pull-requests)
3. [Setup Instructions](#setup-instructions)
4. [Usage Guide](#usage-guide)
5. [Workflow Configuration](#workflow-configuration)
6. [Troubleshooting](#troubleshooting)

---

## Overview

This document explains how Claude Code AI is integrated into the MultiGo Navigation repository using GitHub Actions. Claude provides automated code reviews, interactive Q&A, and intelligent suggestions on pull requests across all branches.

### What is Claude Code?

Claude Code is an AI-powered code review assistant that:
- Automatically reviews code changes in pull requests
- Identifies bugs, security issues, and code quality problems
- Suggests improvements and best practices
- Answers questions about your code interactively
- Provides line-by-line feedback and recommendations

---

## How Claude Works in Pull Requests

### Complete Workflow

```
1. Developer creates/updates a Pull Request
   ↓
2. GitHub detects the PR event (opened, synchronized, reopened)
   ↓
3. GitHub Actions triggers the Claude Code workflow
   ↓
4. Workflow executes: anthropics/claude-code-action@v1
   ↓
5. Claude API connects and analyzes code changes:
   - Reads the diff (what files changed)
   - Reviews code quality and structure
   - Checks for bugs and security vulnerabilities
   - Analyzes best practices compliance
   - Understands context from commit messages
   ↓
6. Claude posts review comments directly on the PR
   ↓
7. Developer can reply to Claude's comments
   ↓
8. Claude responds to questions and provides clarifications
```

### What Claude Does Automatically

#### 1. Automated Code Review
When a PR is created, Claude will:
- Review all code changes in the diff
- Identify potential bugs and logic errors
- Detect security vulnerabilities
- Check for code style inconsistencies
- Suggest performance optimizations
- Recommend ROS2 best practices

**Example:**
```python
# Your code in PR:
def calculate_velocity(distance, time):
    return distance / time

# Claude's comment:
⚠️ Potential division by zero error detected!

Recommendation:
def calculate_velocity(distance, time):
    if time == 0:
        raise ValueError("Time cannot be zero")
    return distance / time
```

#### 2. Interactive Q&A
You can ask Claude questions by mentioning it in PR comments.

**Example conversation:**
```
Developer: @claude Can you explain the RTABMap configuration changes?

Claude: This PR modifies RTABMap SLAM configuration:
1. Updated subscribe_depth parameter for better 3D mapping
2. Increased queue_size to 10 for reliability
3. Added frame_id specification for proper TF transforms
4. These changes improve localization accuracy in indoor environments
```

#### 3. Code Suggestions
Claude provides actionable code improvements:

```diff
# Original code
- ros2 launch boot simulation.launch.py

# Claude's suggestion
+ ros2 launch boot simulation.launch.py --use-sim-time true
+ # This ensures proper simulation time synchronization
```

#### 4. Line-by-line Comments
Claude comments on specific code lines:

```yaml
# Line 45 in navigation.launch.py
use_sim_time: True

# Claude's comment on this line:
💡 Consider making this configurable via launch argument:
DeclareLaunchArgument('use_sim_time', default_value='false')
```

#### 5. Security Analysis
Detects security vulnerabilities:

```python
# Your code
password = "hardcoded_password_123"

# Claude's comment
🔒 Security Issue: Hardcoded credentials detected!
Move to environment variables or secure configuration file.
```

---

## Setup Instructions

### Prerequisites
- Admin access to the GitHub repository
- GitHub account with permissions to create Personal Access Tokens

### Step 1: Create GitHub Personal Access Token

1. Navigate to: https://github.com/settings/personal-access-tokens/new

2. Configure the token:
   - **Token name**: `Claude Code Actions`
   - **Expiration**: 90 days (or your preference)
   - **Repository access**: Select "Only select repositories"
     - Choose: `Futu-reADS/multigo_navigation`

3. **Set Repository Permissions** (expand "Repository permissions"):
   - ✅ **Actions**: Read and write
   - ✅ **Checks**: Read and write
   - ✅ **Contents**: Read and write
   - ✅ **Metadata**: Read-only (automatic)
   - ✅ **Pull requests**: Read and write
   - ✅ **Workflows**: Read and write

4. Click **"Generate token"**

5. **IMPORTANT**: Copy the token immediately (starts with `github_pat_...`)
   - You won't be able to see it again!

### Step 2: Add Token as Repository Secret

1. Go to repository secrets page:
   ```
   https://github.com/Futu-reADS/multigo_navigation/settings/secrets/actions
   ```

2. Click **"New repository secret"**

3. Configure the secret:
   - **Name**: `CLAUDE_CODE_TOKEN` (must be exactly this name)
   - **Secret**: Paste the token you copied in Step 1

4. Click **"Add secret"**

### Step 3: Verify Workflow Installation

The workflow file is already installed at:
```
.github/workflows/claude-code.yml
```

To verify:
1. Go to: https://github.com/Futu-reADS/multigo_navigation/actions
2. You should see **"Claude Code Integration"** in the workflows list

### Step 4: Test the Integration

Create a test pull request to verify everything works:

1. Make a small change in any branch
2. Create a PR to `main`, `develop`, or another `feature/*` branch
3. Check the **"Checks"** tab in the PR
4. You should see "Claude Code Integration" running
5. Claude will post review comments within a few minutes

---

## Usage Guide

### Supported Branching Strategy

The workflow is configured to work with this branching model:
```
main → develop → feature/* branches
```

**Workflow triggers on:**
- Pull requests targeting: `main`, `develop`, or any `feature/*` branch
- Pushes to: `main`, `develop`, or any `feature/*` branch
- Comments on issues and pull requests

### Interacting with Claude

#### Method 1: Automatic Review
Simply create a PR. Claude will automatically review it and post comments.

#### Method 2: Ask Questions
Mention Claude in PR comments:
```
@claude Can you explain the navigation stack changes in this PR?
```

#### Method 3: Request Specific Reviews
```
@claude Please review the RTABMap configuration for performance issues
```

#### Method 4: Ask for Code Examples
```
@claude Can you show me how to properly initialize the TF listener?
```

#### Method 5: Request Explanations
```
@claude Why is this change needed for the localization module?
```

### Example Interactions

#### Example 1: ROS2 Best Practices
```
Developer: @claude Is this the correct way to create a ROS2 node?

Claude: Your node initialization looks good, but consider these improvements:

1. Add parameter declarations for configurability:
   self.declare_parameter('update_rate', 10.0)

2. Use get_logger() for consistent logging:
   self.get_logger().info('Node started')

3. Add proper cleanup in destroy_node():
   def destroy_node(self):
       self.cleanup_resources()
       super().destroy_node()
```

#### Example 2: Navigation Stack Review
```
Developer: @claude Review the navigation parameter changes

Claude: Navigation parameter analysis:

✅ Good changes:
- Increased planner_patience for better path planning
- Updated controller_frequency for smoother motion

⚠️ Suggestions:
1. Consider reducing max_vel_x to 0.5 for indoor safety
2. Add recovery_behaviors configuration
3. Increase transform_tolerance to 0.5 for stability

Example configuration:
max_vel_x: 0.5
recovery_behaviors: ['rotate_recovery', 'clear_costmap_recovery']
transform_tolerance: 0.5
```

#### Example 3: Bug Detection
```
Developer: Creates PR with code change

Claude: (Automatically comments)

🐛 Potential bug detected in src/localization/amcl_node.py:45

Issue: Unhandled exception when TF transform fails

Current code:
transform = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())

Suggested fix:
try:
    transform = self.tf_buffer.lookup_transform(
        'map', 'base_link',
        rclpy.time.Time(),
        timeout=rclpy.duration.Duration(seconds=1.0)
    )
except tf2_ros.TransformException as ex:
    self.get_logger().warn(f'Could not transform: {ex}')
    return
```

---

## Workflow Configuration

### Current Configuration

The workflow is defined in `.github/workflows/claude-code.yml`:

```yaml
name: Claude Code Integration

on:
  pull_request:
    types: [opened, synchronize, reopened]
    branches:
      - main
      - develop
      - 'feature/**'
  pull_request_target:
    types: [opened, synchronize, reopened]
    branches:
      - main
      - develop
      - 'feature/**'
  issue_comment:
    types: [created]
  push:
    branches:
      - main
      - develop
      - 'feature/**'

permissions:
  contents: write
  pull-requests: write
  issues: write
  checks: write
  statuses: write

jobs:
  claude-code:
    runs-on: ubuntu-latest
    steps:
      - name: Checkout code
        uses: actions/checkout@v4

      - name: Claude Code Action
        uses: anthropics/claude-code-action@v1
        with:
          github-token: ${{ secrets.CLAUDE_CODE_TOKEN }}
```

### Configuration Breakdown

#### Triggers (`on:`)

**1. Pull Request Events**
```yaml
pull_request:
  types: [opened, synchronize, reopened]
  branches: [main, develop, 'feature/**']
```
- `opened`: New PR created → Claude reviews automatically
- `synchronize`: PR updated with new commits → Claude reviews changes
- `reopened`: Closed PR reopened → Claude re-reviews

**2. Pull Request Target Events**
```yaml
pull_request_target:
  types: [opened, synchronize, reopened]
```
- Allows Claude to review PRs from forks (if you accept external contributions)

**3. Issue Comments**
```yaml
issue_comment:
  types: [created]
```
- Allows Claude to respond when you ask questions in comments

**4. Push Events**
```yaml
push:
  branches: [main, develop, 'feature/**']
```
- Claude can analyze direct commits to these branches

#### Permissions
```yaml
permissions:
  contents: write        # Read code and write analysis results
  pull-requests: write   # Post review comments on PRs
  issues: write          # Comment on issues
  checks: write          # Report check status
  statuses: write        # Update commit statuses
```

#### Job Definition
```yaml
jobs:
  claude-code:
    runs-on: ubuntu-latest  # Runs on GitHub's Ubuntu server
    steps:
      - name: Checkout code
        uses: actions/checkout@v4  # Downloads your repository code

      - name: Claude Code Action
        uses: anthropics/claude-code-action@v1  # Runs Claude analysis
        with:
          github-token: ${{ secrets.CLAUDE_CODE_TOKEN }}  # Authenticates
```

### Supported Branch Workflows

#### Scenario 1: Feature to Develop
```bash
feature/localization → develop (PR)
```
✅ Claude reviews automatically

#### Scenario 2: Develop to Main
```bash
develop → main (PR)
```
✅ Claude reviews automatically

#### Scenario 3: Feature to Main
```bash
feature/new-feature → main (PR)
```
✅ Claude reviews automatically

#### Scenario 4: Direct Push
```bash
git push origin develop
```
✅ Claude analyzes the commit

---

## Troubleshooting

### Issue 1: Workflow Not Appearing in Actions Tab

**Symptom**: Can't see "Claude Code Integration" at https://github.com/Futu-reADS/multigo_navigation/actions

**Solutions**:
1. Verify workflow file exists on the branch:
   ```bash
   git ls-tree -r origin/main --name-only | grep claude-code.yml
   ```

2. Check if workflow is on `main` branch (required for visibility)

3. Look for syntax errors in the YAML file

### Issue 2: Workflow Fails with Authentication Error

**Symptom**: Workflow runs but fails with "authentication failed" or "401 Unauthorized"

**Solutions**:
1. Verify `CLAUDE_CODE_TOKEN` secret exists:
   - Go to: Settings → Secrets and variables → Actions
   - Confirm `CLAUDE_CODE_TOKEN` is listed

2. Check token permissions:
   - Token must have all required permissions (see Step 1 of setup)

3. Check token expiration:
   - Regenerate if expired

4. Verify secret name is exactly `CLAUDE_CODE_TOKEN` (case-sensitive)

### Issue 3: Claude Doesn't Respond to Comments

**Symptom**: You mention `@claude` but get no response

**Solutions**:
1. Ensure you're mentioning in a PR comment, not a code review comment

2. Check the Actions tab for workflow run errors

3. Verify the workflow has `issue_comment` trigger enabled

4. Wait a few minutes - Claude may be processing

### Issue 4: Workflow Not Triggering on PR

**Symptom**: Created PR but workflow doesn't run

**Solutions**:
1. Verify PR targets a supported branch (`main`, `develop`, or `feature/*`)

2. Check if workflow file exists on the **target** branch

3. Look for YAML syntax errors:
   ```bash
   # Validate YAML syntax
   python3 -c "import yaml; yaml.safe_load(open('.github/workflows/claude-code.yml'))"
   ```

4. Check GitHub Actions are enabled for the repository

### Issue 5: Private Repository Issues

**Symptom**: Workflow fails with "repository not found" or similar

**Solutions**:
1. For private repos, ensure the Personal Access Token has access to the repository

2. Verify repository selection when creating the token

3. Check organization settings allow GitHub Actions

### Issue 6: Rate Limiting

**Symptom**: Workflow fails with "rate limit exceeded"

**Solutions**:
1. Wait for rate limit to reset (usually 1 hour)

2. Consider reducing workflow triggers (e.g., remove `push` events)

3. Use `pull_request` instead of `pull_request_target` to reduce API calls

### Getting Help

If you encounter issues not covered here:

1. Check workflow run logs:
   - Go to Actions tab
   - Click on failed workflow run
   - Review logs for error messages

2. Verify GitHub Actions status:
   - https://www.githubstatus.com/

3. Review Claude Code documentation:
   - https://github.com/anthropics/claude-code-action

4. Check repository settings:
   - Settings → Actions → General
   - Ensure "Allow all actions and reusable workflows" is selected

---

## Best Practices

### 1. Use Descriptive PR Titles and Descriptions
Claude uses PR context to provide better reviews. Include:
- What changed and why
- Related issue numbers
- Testing performed

### 2. Ask Specific Questions
Instead of: `@claude Review this`
Try: `@claude Can you check if the TF transforms are correct in the navigation stack?`

### 3. Respond to Claude's Suggestions
Engage with Claude's comments to get clarifications or ask follow-up questions.

### 4. Keep PRs Focused
Smaller, focused PRs get better reviews than large, sprawling changes.

### 5. Regular Token Rotation
Rotate the Personal Access Token every 90 days for security.

### 6. Monitor Workflow Costs
GitHub Actions have usage limits. Monitor at:
```
Settings → Billing → Actions usage
```

---

## Security Considerations

### Token Security
- ✅ **DO**: Store token as a GitHub secret
- ✅ **DO**: Use fine-grained tokens with minimal permissions
- ✅ **DO**: Set token expiration
- ❌ **DON'T**: Commit tokens to the repository
- ❌ **DON'T**: Share tokens with others
- ❌ **DON'T**: Use classic tokens (use fine-grained instead)

### Code Privacy
- Claude analyzes code sent through the API
- For sensitive code, review Anthropic's privacy policy
- Consider limiting Claude to non-sensitive repositories

### Workflow Security
- Workflow runs in isolated GitHub Actions environment
- No access to production systems
- Read-only access to code by default

---

## Additional Resources

- **Claude Code Documentation**: https://github.com/anthropics/claude-code-action
- **GitHub Actions Documentation**: https://docs.github.com/en/actions
- **ROS2 Best Practices**: https://docs.ros.org/en/humble/
- **Repository**: https://github.com/Futu-reADS/multigo_navigation

---

## Changelog

### 2025-11-18
- Initial setup of Claude Code integration
- Configured workflow for main, develop, and feature branches
- Added comprehensive documentation
- Configured multi-level branch support (main → develop → feature/*)

---

## License

This integration setup follows the same license as the main repository.

---

**Questions?** Ask Claude directly in any PR: `@claude [your question]`
