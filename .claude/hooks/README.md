# Claude Code Hooks - Complete System

**Automatic tracking of all Claude Code sessions with GitHub Issue + PR comments**

---

## 🎯 System Overview

This system meets your requirements:

1. ✅ **Save all prompts locally** (for senior management review)
2. ✅ **Post comments to GitHub Issues** (during implementation)
3. ✅ **Post comments to PRs** (during code review)
4. ✅ **Multiple iterations = Multiple comments** (on both Issue and PR)
5. ✅ **Smart branch detection** (feature/5-login → Issue #5)

---

## 📋 Three-Part System

### Part 1: Local Prompt Logging
**Purpose:** Permanent record for senior management

**File:** `docs/dev-logs/issue-{N}.md`
- Logs every prompt you ask
- Organized by issue number (from branch name)
- Committed to git (permanent audit trail)

### Part 2: GitHub Issue Comments
**Purpose:** Track implementation progress

**Posted to:** Issue #N (extracted from branch)
- Shows what you asked Claude Code
- Shows what files were created/edited
- Shows duration and tools used
- Multiple sessions = Multiple comments

### Part 3: GitHub PR Comments
**Purpose:** Help code reviewers

**Posted to:** PR (when it exists)
- Same information as Issue comments
- Formatted for code review context
- Works alongside @claude automatic reviews

---

## 🎬 Complete Workflow Example

### Scenario: Issue #5 "Add User Login"

```bash
# 1. Issue exists on GitHub
# https://github.com/yourrepo/issues/5

# 2. Start work (branch name is KEY!)
git checkout -b feature/5-user-login
#                      ↑
#                  Extracts issue #5

# 3. Ask Claude Code
"Please implement JWT authentication with login endpoint"

# Behind the scenes:
# ✓ Logged to: docs/dev-logs/issue-5.md
# ✓ Session tracking starts

# 4. Claude creates files
# Write: src/auth/jwt.js
# Edit: src/routes/auth.js
# (2 tools = threshold reached!)

# ✓ Comment posted to Issue #5 automatically!
```

**Issue #5 now shows:**
```markdown
## 💻 Claude Code Implementation Update

**Time:** 2025-11-21 14:30:00 | **Duration:** 2m

### 📝 Prompt Asked
```
Please implement JWT authentication with login endpoint
```

### ✅ What Was Done
**Files Created (1):**
- `src/auth/jwt.js`

**Files Edited (1):**
- `src/routes/auth.js`

### 📊 Summary
- **Total files modified:** 2
- **Tools used:** 2
- **Duration:** 2m

---
*🤖 Automated update from Claude Code - Implementation in progress*
```

```bash
# 5. Continue working
"Add password hashing with bcrypt"

# ✓ New comment on Issue #5!

# 6. Create PR
gh pr create --title "Add user login" --body "Resolves #5"
# Creates PR #8

# 7. More work
"Add login tests"

# ✓ Comment on Issue #5 (still tracking implementation)
# ✓ Comment on PR #8 (now tracking review too!)

# Result:
# - Issue #5: Has 3 comments (all implementation steps)
# - PR #8: Has 1 comment (latest session)
# - docs/dev-logs/issue-5.md: Has all prompts saved
```

---

## 📁 Files in This Directory

```
.claude/hooks/
├── README.md                          ← This file
├── settings.json (parent dir)         ← Hook configuration
├── user-prompt-submit-enhanced.sh     ← Captures prompts
├── post-tool-use.sh                   ← Captures tool usage
├── prompt-logger.js                   ← Logs to local files
├── github-commenter.js                ← Posts to Issue + PR ⭐
├── pr-commenter.js                    ← Old (not used)
└── test-hook.sh                       ← Test script
```

**Key File:** `github-commenter.js` - The unified commenter that:
- Extracts issue number from branch name
- Posts to Issue (always)
- Posts to PR (if exists)
- Keeps local logs (backup)

---

## ⚙️ Configuration

### Current Settings (`.claude/settings.json`)

```json
{
  "hooks": {
    "UserPromptSubmit": [
      {
        "hooks": [
          {
            "type": "command",
            "command": ".claude/hooks/user-prompt-submit-enhanced.sh"
          }
        ]
      }
    ],
    "PostToolUse": [
      {
        "matcher": {
          "tool_name": "(Write|Edit|Bash)"
        },
        "hooks": [
          {
            "type": "command",
            "command": ".claude/hooks/post-tool-use.sh"
          }
        ]
      }
    ]
  }
}
```

### Branch Naming Convention

For automatic issue detection, use:
```
feature/5-description   → Issue #5
fix/123-bug-name       → Issue #123
issue-42               → Issue #42
refactor/7-cleanup     → Issue #7
```

**Pattern:** `(feature|fix|issue|refactor|chore)/{NUMBER}-description`

---

## 🎨 Comment Formats

### On Issues (Implementation Tracking)
```markdown
## 💻 Claude Code Implementation Update

**Time:** 2025-11-21 14:30:00 | **Duration:** 2m

### 📝 Prompt Asked
```
Your question here
```

### ✅ What Was Done
**Files Created (2):**
- file1.js
- file2.js

**Files Edited (1):**
- existing-file.js

**Commands Run (1):**
- npm test

### 📊 Summary
- **Total files modified:** 3
- **Tools used:** 3
- **Duration:** 2m

---
*🤖 Automated update from Claude Code - Implementation in progress*
```

### On PRs (Code Review Context)
```markdown
## 🤖 Claude Code Session Summary

**Time:** 2025-11-21 14:30:00 (2m)

### 📝 Prompt Asked
```
Your question here
```

### ✅ What Was Done
[Same content as Issue comment]

### 📊 Summary
[Same summary]

---
*Auto-generated by Claude Code*
```

---

## 🧪 Testing

### Test Prompt Logging

```bash
./.claude/hooks/test-hook.sh
cat docs/dev-logs/*.md
```

### Test Issue + PR Commenting

```bash
# 1. Create test issue
gh issue create --title "Test automation" --body "Testing hooks"
# Creates Issue #10

# 2. Create branch (use issue number!)
git checkout -b feature/10-test

# 3. Simulate prompt
echo '{"hook_event_name":"UserPromptSubmit","prompt":"Test implementation with multiple files"}' | ./.claude/hooks/user-prompt-submit-enhanced.sh

# 4. Simulate tools (need 2 for threshold)
echo '{"hook_event_name":"PostToolUse","tool_name":"Write","file_path":"test1.js"}' | ./.claude/hooks/post-tool-use.sh

echo '{"hook_event_name":"PostToolUse","tool_name":"Edit","file_path":"test2.js"}' | ./.claude/hooks/post-tool-use.sh

# 5. Check Issue #10 for comment!
gh issue view 10 --comments

# 6. Create PR and test again
gh pr create --title "Test" --body "Resolves #10"

# 7. Simulate another session
echo '{"hook_event_name":"UserPromptSubmit","prompt":"Add tests"}' | ./.claude/hooks/user-prompt-submit-enhanced.sh

echo '{"hook_event_name":"PostToolUse","tool_name":"Write","file_path":"test.test.js"}' | ./.claude/hooks/post-tool-use.sh

echo '{"hook_event_name":"PostToolUse","tool_name":"Bash","command":"npm test"}' | ./.claude/hooks/post-tool-use.sh

# 8. Check both Issue #10 AND PR for comments!
gh issue view 10 --comments
gh pr view --comments
```

---

## 🐛 Troubleshooting

### Comments Not Posting to Issue

**Check 1: Branch naming**
```bash
git branch --show-current
# Should be: feature/5-something, fix/123-bug, etc.
```

**Check 2: Issue exists**
```bash
gh issue view 5
# Should show the issue
```

**Check 3: GitHub CLI authenticated**
```bash
gh auth status
# Should show: Logged in to github.com
```

### Comments Not Posting to PR

**Check 1: PR exists for branch**
```bash
gh pr list --head $(git branch --show-current)
# Should show your PR
```

### Only Getting Issue Comments, Not PR

**This is EXPECTED until you create the PR!**
- Issue comments: Start immediately when you create branch
- PR comments: Start only after `gh pr create`

### Multiple Comments on Issue

**This is INTENTIONAL!**
- Each Claude Code session = 1 comment
- Multiple questions = Multiple comments
- This tracks your complete implementation journey

---

## 💡 Customization

### Change Tools That Trigger Comments

Edit `.claude/hooks/github-commenter.js`:
```javascript
const CONFIG = {
  significantTools: ['Write', 'Edit', 'Bash', 'NotebookEdit'],  // Add more
  minToolsThreshold: 2,  // Change to 1 for more frequent comments
};
```

### Change Comment Threshold

Default: Posts after 2 tools used

To make it 3 tools:
```javascript
minToolsThreshold: 3,
```

To post after every tool:
```javascript
minToolsThreshold: 1,
```

### Disable Issue Comments (PR only)

Edit `.claude/hooks/github-commenter.js`, find the posting logic and comment out:
```javascript
// if (issueNumber) {
//   postGitHubComment('issue', issueNumber, comment);
// }
```

### Disable PR Comments (Issue only)

```javascript
// if (prNumber) {
//   postGitHubComment('pr', prNumber, comment);
// }
```

---

## 📊 How It All Works Together

```
┌─────────────────────────────────────────────────┐
│          You ask Claude Code a question         │
└──────────────────┬──────────────────────────────┘
                   │
                   ▼
         UserPromptSubmit Hook
                   │
        ┌──────────┴──────────┐
        │                     │
        ▼                     ▼
  prompt-logger.js    github-commenter.js
  (local files)       (session tracking)
        │                     │
        ▼                     │
  docs/dev-logs/              │
  issue-N.md                  │
  (saved for senior)          │
                              │
                   Claude uses tools (Write, Edit, Bash)
                              │
                              ▼
                     PostToolUse Hook
                              │
                              ▼
                    github-commenter.js
                    (tracks tool usage)
                              │
                   After 2+ tools threshold
                              │
                 ┌────────────┴────────────┐
                 │                         │
                 ▼                         ▼
         Extract issue from       Check if PR exists
         branch name              for this branch
         (feature/5-x → #5)              │
                 │                       │
                 ▼                       ▼
         Post to Issue #5        Post to PR (if exists)
         "Implementation          "Session Summary"
          Update"                        │
                 │                       │
                 └───────────┬───────────┘
                             │
                             ▼
                  Clear session, start new one
```

---

## 🎓 Best Practices

### Branch Naming
✅ **DO:** `feature/5-user-login`, `fix/123-auth-bug`
❌ **DON'T:** `my-feature`, `test-branch`

### Issue Numbers
- Always create GitHub Issue first
- Use issue number in branch name
- One branch per issue

### Multiple Sessions
- Each meaningful change = New prompt
- Let Claude Code finish before next prompt
- Comments track your complete journey

### PR Timing
- Create PR when ready for review
- Comments start appearing on PR too
- Both Issue and PR get updates

---

## 🔗 Related Documentation

**In This Repo:**
- `docs/dev-logs/README.md` - Log directory usage
- `README.md` - Main project documentation

**In workflow-system:**
- `/home/pankaj/workflow-system/HOOKS-SETUP.md` - Original hook setup
- `/home/pankaj/workflow-system/PROMPT-COMPATIBILITY.md` - How @claude prompts work

---

## ✅ Verification Checklist

After setup, verify:

- [ ] `.claude/settings.json` has both hooks configured
- [ ] `.claude/hooks/github-commenter.js` exists and is executable
- [ ] `docs/dev-logs/` directory exists
- [ ] GitHub CLI authenticated (`gh auth status`)
- [ ] Branch follows naming convention (feature/N-description)
- [ ] Issue exists on GitHub
- [ ] Test: Ask Claude Code a question, use 2+ tools, check Issue for comment

---

## 📈 What You Get

### For You (Developer)
- Clear tracking of what you built
- Easy reference for what was implemented when
- Automatic documentation

### For Senior Management
- All prompts saved locally in git
- Complete audit trail
- Searchable by issue number

### For Code Reviewers
- PR comments show what Claude Code did
- Context for code changes
- Faster reviews

### For The Team
- Issue comments show implementation progress
- Visible AI-assisted development
- Better collaboration

---

**Status:** ✅ Production Ready
**Tested:** ✅ Live on Issue #4 and PR #5
**Requirements:** ✅ All met

---

*This system provides complete traceability of all Claude Code sessions, satisfying both management requirements (local logs) and team collaboration needs (GitHub comments).*
