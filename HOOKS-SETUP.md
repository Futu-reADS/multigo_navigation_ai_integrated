# 🪝 Hooks Setup Guide

**Updated: Claude Code hooks don't fire - Using working alternative**

---

## ⚠️ Important Update

**Claude Code's built-in hooks (`UserPromptSubmit`, `PostToolUse`) are configured in this template but DON'T WORK** in the current environment.

After extensive testing:
- ✅ Hooks are configured correctly
- ✅ Hook scripts work when tested manually
- ❌ **Claude Code never triggers them during sessions**

**Solution:** Use the manual `post-summary.sh` script instead (see below).

---

## ✅ Working Solution: Manual Summary Script

### Quick Start

1. **Copy files to your project:**
   ```bash
   cp ~/workflow-system/.claude/hooks/post-summary.sh .claude/hooks/
   cp ~/workflow-system/HOW-TO-USE-AUTOMATION.md ./
   chmod +x .claude/hooks/post-summary.sh
   ```

2. **Work normally (commit as much as you want)**

3. **When done, post summary:**
   ```bash
   ./.claude/hooks/post-summary.sh \
     "What you asked Claude to do" \
     "What was accomplished"
   ```

### Example Usage

```bash
# Create branch with issue number
git checkout -b feature/45-add-auth

# Work on task (multiple commits OK)
git commit -m "add auth endpoint"
git commit -m "add tests"
git commit -m "fix bug"

# When DONE, post ONE comprehensive summary:
./.claude/hooks/post-summary.sh \
  "Implement user authentication with JWT tokens" \
  "Added JWT auth with login/register endpoints, bcrypt password hashing, auth middleware for protected routes. Includes 15 tests all passing."

# Output:
# ✓ Posted to Issue #45
# ✓ Posted to PR #50 (if exists)
# ✅ Done!
```

### What Gets Posted

```markdown
## 🔄 Iteration 1

**Time:** 2025-11-21 11:18 (~45m)

### 📝 Request
```
Implement user authentication with JWT tokens
```

### ✅ What Was Added

Added JWT auth with login/register endpoints, bcrypt password
hashing, auth middleware for protected routes. Includes 15 tests
all passing.

### 📁 Files Changed (12 files)
- `src/auth/jwt.js`
- `src/middleware/auth.js`
- `src/routes/auth.js`
...

### 📊 Summary
- **Files changed:** 12
- **Commits:** 5
- **Duration:** ~45m

**Commits:**
```
abc1234 feat: add auth endpoints
def5678 feat: add middleware
ghi9012 test: add auth tests
...
```
```

---

## 📁 Files in Template

### ✅ Working (Use These):
- **`post-summary.sh`** - Manual script for posting GitHub comments
- **`HOW-TO-USE-AUTOMATION.md`** - Complete usage guide

### ❌ Legacy (Don't Work - Kept for Reference):
- `user-prompt-submit-enhanced.sh` - For UserPromptSubmit hook
- `post-tool-use.sh` - For PostToolUse hook
- `github-commenter.js` - Hook backend
- `prompt-logger.js` - Prompt logging
- `settings.json` - Hook configuration

**Note:** Legacy files are kept in case Claude Code hooks start working in future updates.

---

## 🎯 Benefits of Manual System

| Feature | Status |
|---------|--------|
| Works reliably | ✅ |
| Comprehensive summaries | ✅ |
| Manual control | ✅ |
| No spam (not every commit) | ✅ |
| Includes original prompt | ✅ |
| Shows achievement | ✅ |
| Lists all commits | ✅ |
| Lists all files | ✅ |
| Posts to Issue & PR | ✅ |

---

## 🔧 Setup for New Project

```bash
# 1. Create project
mkdir my-project && cd my-project
git init

# 2. Copy automation files
cp ~/workflow-system/.claude/hooks/post-summary.sh .claude/hooks/
cp ~/workflow-system/HOW-TO-USE-AUTOMATION.md ./

# 3. Make executable
chmod +x .claude/hooks/post-summary.sh

# 4. Create branch with issue number
git checkout -b feature/10-my-feature

# 5. Work and commit normally
# ... make changes ...
git commit -m "feat: add feature"

# 6. When done, post summary
./.claude/hooks/post-summary.sh \
  "Your request here" \
  "What was accomplished"
```

---

## 📚 Documentation

- **Quick Guide:** `HOW-TO-USE-AUTOMATION.md`
- **Workflow Overview:** `WORKFLOW.md`
- **Complete Setup:** `README.md`

---

## ❓ FAQ

### Q: Why don't Claude Code hooks work?

**A:** After extensive testing, we found:
1. Hooks are configured correctly in `.claude/settings.json`
2. Hook scripts are executable and work when tested manually
3. Multiple configurations and tests performed
4. **Claude Code simply doesn't call them during tool execution**

This appears to be an environment or platform limitation.

### Q: Will hooks work in the future?

**A:** Possibly! The hook files are kept in the template so if Claude Code fixes this, the system will automatically start working. Until then, use the manual script.

### Q: Can I still use the old hook files?

**A:** They won't trigger automatically, but you can test them manually. The new manual system is more reliable.

### Q: Is the manual script better?

**A:** Yes! Benefits:
- ✅ You control when to post (not spamming after every commit)
- ✅ Post ONE comprehensive summary when done
- ✅ Better format with prompt + achievement
- ✅ No "Session 1, Session 2" spam

### Q: How do I know what to write for prompt/achievement?

**A:**
- **Prompt:** What you asked Claude (copy your original request)
- **Achievement:** Brief summary of what was done (files, features, tests)

**Example:**
```bash
./.claude/hooks/post-summary.sh \
  "Add dark mode toggle to settings page" \
  "Added dark mode toggle component in settings, implemented theme context, updated 8 components to support themes, added localStorage persistence. All 12 tests passing."
```

---

## 🎉 Summary

**Old System (Doesn't Work):**
- ❌ Claude Code hooks don't fire
- ❌ Auto-posting after every commit
- ❌ Session spam

**New System (Works Great):**
- ✅ Manual script that works reliably
- ✅ Post when YOU want
- ✅ Comprehensive meaningful comments
- ✅ Original prompt + achievement

**Setup:** Copy `post-summary.sh` and use it when done with tasks!

---

**Last Updated:** 2025-11-21
**Status:** ✅ Working Solution Available
