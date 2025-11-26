# 🤖 GitHub @claude Integration Setup Guide

**Complete guide to enable @claude bot in your GitHub issues and pull requests**

---

## 📋 What This Does

Enables @claude to:
- ✅ Respond to questions in GitHub issues
- ✅ Provide architecture recommendations
- ✅ Review pull requests automatically
- ✅ Answer questions in PR comments

**Cost:** FREE (uses your Claude Code OAuth token)

---

## 🎯 Prerequisites

Before starting, you need:
1. ✅ Claude Code installed and authenticated
2. ✅ GitHub CLI (`gh`) installed and authenticated
3. ✅ A GitHub repository where you want to enable @claude

---

## 📖 Step-by-Step Setup

### **Step 1: Get Your Claude OAuth Token**

Your Claude Code credentials are stored locally. Extract the OAuth token:

```bash
# The token is in this file
cat ~/.claude/.credentials.json

# Or extract just the access token
cat ~/.claude/.credentials.json | grep -o '"accessToken":"[^"]*"' | cut -d'"' -f4
```

You'll see output like:
```json
{
  "claudeAiOauth": {
    "accessToken": "sk-ant-oat01-xxxxx...",
    "refreshToken": "sk-ant-ort01-xxxxx...",
    ...
  }
}
```

**Copy the `accessToken` value** (starts with `sk-ant-oat01-`)

---

### **Step 2: Add Token to GitHub Repository**

```bash
# Navigate to your repository
cd ~/your-project

# Authenticate with GitHub CLI (if not already)
gh auth login

# Add the token as a GitHub secret
gh secret set CLAUDE_CODE_OAUTH_TOKEN

# When prompted, paste your access token
# (The one starting with sk-ant-oat01-...)
```

**Verify it was added:**
```bash
gh secret list
```

You should see:
```
CLAUDE_CODE_OAUTH_TOKEN  Updated YYYY-MM-DD
```

---

### **Step 3: Verify GitHub Actions Workflows**

This template includes optimized workflow files in `.github/workflows/`:

**✅ Already included:**
- `claude.yml` - Interactive @claude bot with safety features
- `claude-code-review.yml` - Automatic PR reviews with size limits

**Features:**
- ✅ 30-minute timeout protection
- ✅ Concurrency control (prevents spam)
- ✅ Failure notifications
- ✅ Full git history access
- ✅ PR size checking (auto-review only for PRs <50 files/<2000 lines)
- ✅ Smart file filtering (excludes lock files)

**Verify they exist:**
```bash
ls -la .github/workflows/
# Should show:
# - claude.yml
# - claude-code-review.yml
```

**No need to create manually!** These files are already configured and production-ready.

---

### **Step 4: Commit and Push**

```bash
# Add the workflow file
git add .github/workflows/claude.yml

# Commit
git commit -m "feat: add GitHub @claude integration"

# Push to GitHub
git push origin main
```

---

### **Step 5: Enable GitHub Actions**

1. **Open your repository on GitHub:**
   ```
   https://github.com/YOUR_USERNAME/YOUR_REPO
   ```

2. **Go to the "Actions" tab**

3. **If you see "Workflows need permission to run":**
   - Click **"I understand my workflows, go ahead and enable them"**

4. You should now see **"Claude Code"** listed as a workflow

---

### **Step 6: Test It!**

1. **Go to any issue in your repository**

2. **Add a comment mentioning @claude:**
   ```markdown
   @claude Can you help me design the architecture for this feature?
   ```

3. **Wait 30-60 seconds**

4. **Check the Actions tab** - you should see the workflow running

5. **@claude should respond to your comment!** 🎉

---

## 🔧 How It Works

### Workflow Triggers

The workflow runs when:
- ✅ Someone creates a comment on an issue (with `@claude`)
- ✅ Someone creates a comment on a PR (with `@claude`)
- ✅ Someone submits a PR review (with `@claude`)
- ✅ A new issue is opened (with `@claude` in title or body)

### The `if` Condition

```yaml
if: |
  (github.event_name == 'issue_comment' && contains(github.event.comment.body, '@claude'))
```

This ensures the workflow **only runs when @claude is mentioned**, saving compute time and costs.

### Permissions

```yaml
permissions:
  contents: write        # Read repository code
  pull-requests: write   # Comment on PRs
  issues: write          # Comment on issues
  id-token: write        # OAuth authentication
  actions: read          # Read CI results
```

These permissions allow @claude to:
- Read your code
- Post comments
- View test results

---

## 🎨 Usage Examples

### Architecture Discussion

```markdown
@claude I'm building a REST API for task management. What's the best architecture?

Requirements:
- Node.js + Express
- In-memory storage (for now)
- CRUD operations for tasks
- Input validation

Please provide recommendations for:
1. Project structure
2. Error handling
3. Validation approach
```

### Code Review

```markdown
@claude Please review this PR for:
- Security issues
- Performance problems
- Code quality
- Best practices
```

### Debugging Help

```markdown
@claude I'm getting this error:

```
TypeError: Cannot read property 'id' of undefined
```

The code is in src/controllers/taskController.js

Can you help identify the issue?
```

---

## 🔍 Troubleshooting

### Workflow Not Running

**Problem:** @claude doesn't respond

**Solutions:**

1. **Check Actions tab** - Is the workflow running?
   ```
   https://github.com/YOUR_USERNAME/YOUR_REPO/actions
   ```

2. **Verify secret is set:**
   ```bash
   gh secret list
   ```

3. **Check workflow syntax:**
   ```bash
   cat .github/workflows/claude.yml
   ```

4. **Enable Actions** if disabled:
   - Go to Settings → Actions → General
   - Enable "Allow all actions"

---

### Authentication Errors

**Problem:** Workflow fails with "Authentication failed"

**Solutions:**

1. **Regenerate token:**
   ```bash
   # Get fresh token
   cat ~/.claude/.credentials.json | grep accessToken

   # Update GitHub secret
   gh secret set CLAUDE_CODE_OAUTH_TOKEN
   ```

2. **Check token format:**
   - Should start with `sk-ant-oat01-`
   - Should be the `accessToken`, not `refreshToken`

---

### YAML Syntax Errors

**Problem:** "Invalid workflow file" error

**Solutions:**

1. **Validate YAML syntax:**
   ```bash
   # Check for indentation issues
   cat .github/workflows/claude.yml
   ```

2. **Use exact template** from Step 3 above

3. **No leading spaces** - YAML is indentation-sensitive

---

### @claude Responds But Doesn't Help

**Problem:** @claude responds but doesn't provide useful information

**Solutions:**

1. **Be specific in your questions:**
   - ❌ "@claude help"
   - ✅ "@claude design REST API architecture for task management"

2. **Provide context:**
   - Tech stack
   - Requirements
   - Constraints
   - What you've already tried

3. **Ask clear questions:**
   - What should the project structure be?
   - How should I handle errors?
   - What's the security concern here?

---

## 📊 Cost & Limits

### Free Tier

Using your Claude Code OAuth token is **FREE** because:
- It uses your existing Claude Code subscription
- No separate API charges
- Part of your Claude MAX/Code access

### Rate Limits

- **Workflow timeout:** 30 minutes per run
- **Concurrent runs:** Limited by GitHub Actions (usually 20)
- **Token validity:** Tokens refresh automatically

---

## 🔐 Security Notes

### Token Security

1. **Never commit tokens to git**
   - Always use GitHub Secrets
   - Never hardcode in workflow files

2. **Token permissions:**
   - OAuth token has same access as your Claude Code
   - Scoped to your account only

3. **Rotate tokens regularly:**
   ```bash
   # If token is compromised, get a new one
   cat ~/.claude/.credentials.json
   gh secret set CLAUDE_CODE_OAUTH_TOKEN
   ```

### Repository Access

The workflow can:
- ✅ Read your code (as configured)
- ✅ Post comments
- ❌ Push code (not enabled by default)
- ❌ Delete branches (not enabled)

---

## 🎯 Advanced Configuration

### Custom Prompts

You can add custom instructions to @claude:

```yaml
- name: Run Claude Code
  uses: anthropics/claude-code-action@v1
  with:
    claude_code_oauth_token: ${{ secrets.CLAUDE_CODE_OAUTH_TOKEN }}
    prompt: |
      You are reviewing code for a production system.
      Be extra careful about security and performance.
      Follow our coding standards in CONTRIBUTING.md
```

### Restrict to Specific Files

Only trigger on certain file changes:

```yaml
on:
  pull_request:
    paths:
      - 'src/**/*.js'
      - 'src/**/*.ts'
```

### Filter by PR Author

Only run for external contributors:

```yaml
jobs:
  claude:
    if: |
      github.event.pull_request.author_association == 'FIRST_TIME_CONTRIBUTOR'
```

---

## 📚 Additional Resources

- **Claude Code Action:** https://github.com/anthropics/claude-code-action
- **GitHub Actions Docs:** https://docs.github.com/actions
- **Workflow Syntax:** https://docs.github.com/actions/reference/workflow-syntax-for-github-actions

---

## ✅ Quick Setup Checklist

Use this checklist when setting up @claude:

- [ ] Get OAuth token from `~/.claude/.credentials.json`
- [ ] Add token to GitHub: `gh secret set CLAUDE_CODE_OAUTH_TOKEN`
- [ ] Create `.github/workflows/claude.yml`
- [ ] Commit and push workflow file
- [ ] Enable GitHub Actions in repository
- [ ] Test by mentioning @claude in an issue
- [ ] Verify @claude responds within 60 seconds

---

## 🎉 You're Done!

@claude is now active in your repository!

**Next steps:**
- Create issues and ask @claude for help
- Tag @claude in PRs for code reviews
- See the complete workflow in action

**Remember:** This is FREE with your Claude Code access! Use it liberally! 🚀

---

*Part of the Ultimate Development Workflow System*
