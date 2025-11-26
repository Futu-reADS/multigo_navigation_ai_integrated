# CLAUDE.md

**This file provides guidance to Claude Code when working with code in this repository.**

---

## ⚠️ CRITICAL: Read This FIRST - Every Single Time

**BEFORE doing ANYTHING else in this session, you MUST:**

```
1. Read: .claude/WORKFLOW.md (Complete workflow - single source of truth)
2. Understand: What phase of the workflow are we in?
3. Ask: Which step should I follow based on user's request?
```

**The workflow file contains:**
- Complete development process (specification → implementation → PR → merge)
- Automation scripts and when to use them
- Commit standards and comment formats
- Testing requirements
- Common commands and troubleshooting

**DO NOT proceed with any task until you've read `.claude/WORKFLOW.md` and understand the current workflow phase.**

---

## Project Overview

**Repository:** multigo_navigation_ai_integrated
**Type:** ROS 2 Navigation System with AI Integration
**Main Branch:** `main`
**Test Command:** `colcon test` (must pass before commits)

---

## Development Workflow Summary

**The complete workflow is in `.claude/WORKFLOW.md`. Here's the quick reference:**

### Phase 1: Specification & Review
1. Issue created with feature specification
2. @claude reviews specification (GitHub Actions)
3. ClaudeCode reviews @claude's review
4. Discussion/debate to reach consensus
5. Final consensus documented
6. Ready to implement ✅

### Phase 2: Implementation
1. Start work: `./scripts/start-work.sh {issue-number}`
2. Implement with ClaudeCode
3. Test: `colcon test` (must pass)
4. Commit: Follow format in WORKFLOW.md
5. Post progress: `./.claude/hooks/post-summary.sh`

### Phase 3: Pull Request
1. Push branch: `git push -u origin feature/{issue-number}--{description}`
2. Create PR with `gh pr create`
3. @claude reviews PR (GitHub Actions)
4. Address feedback and post updates
5. Merge when approved

### Phase 4: Cleanup
1. Run: `./scripts/cleanup-after-merge.sh`

---

## Key Principles

### When User Gives You a Task:

**Step 1: Check Workflow Phase**
```
Ask yourself:
- Are we in specification phase? (Review and discuss)
- Are we in implementation phase? (Code and test)
- Are we in PR phase? (Address review feedback)
- Are we in cleanup phase? (Merge and clean up)
```

**Step 2: Follow the Workflow**
```
Don't just start coding! Check .claude/WORKFLOW.md for:
- What should I do in this phase?
- What commands should I run?
- What format should I use?
- What's the next step after this?
```

**Step 3: Ask If Unclear**
```
If the user's request doesn't clearly map to a workflow phase:
- Ask which phase we're in
- Clarify what the goal is
- Confirm the correct step before proceeding
```

---

## Architecture & Code Standards

### Project Structure
```
src/
├── models/         # Data models (Task.js)
├── controllers/    # Request handlers (taskController.js)
├── middleware/     # Validation, error handling
├── routes/         # API routes (tasks.js)
└── app.js          # Express app setup

tests/
└── tasks.test.js   # Comprehensive test suite
```

### Code Patterns

**Model Layer (src/models/Task.js):**
- In-memory storage (tasks array)
- Static methods for operations
- Example: `Task.findAll()`, `Task.findById()`, `Task.create()`

**Controller Layer (src/controllers/taskController.js):**
- Request handling
- Response format: `{ success: true, data: {...} }`
- Error handling with try/catch and next(error)

**Validation (src/middleware/validation.js):**
- express-validator for input validation
- Middleware functions that validate and sanitize
- Return 400 errors with validation details

**Testing (tests/tasks.test.js):**
- Jest + Supertest
- Comprehensive coverage (aim for 80%+)
- Test structure: describe blocks for endpoints
- Always clean up with `beforeEach`

---

## Critical Rules

### ✅ DO:
- Read `.claude/WORKFLOW.md` at the start of EVERY session
- Ask which workflow phase we're in if unclear
- Follow the workflow step-by-step
- Run `colcon test` before every commit
- Use `.claude/hooks/post-summary.sh` after milestone commits
- Reference specific files and line numbers in comments
- Follow commit message format from WORKFLOW.md

### ❌ DON'T:
- Start coding without checking the workflow
- Skip testing before commits
- Create PRs with failing tests
- Ignore the automation scripts
- Write vague commit messages or comments
- Commit without understanding which phase we're in

---

## Automation Scripts

**All detailed in `.claude/WORKFLOW.md`**

- **post-summary.sh** - Post progress comments to GitHub
- **start-work.sh** - Create feature branch
- **cleanup-after-merge.sh** - Clean up after PR merge

**GitHub Actions:**
- **@claude bot** - Reviews specifications and PRs (review-only mode)
- **ClaudeCode (you)** - Does actual implementation

**Separation of Roles:**
- @claude = Reviewer/Advisor (no code changes)
- ClaudeCode = Implementer (writes code)

---

## Before Every Response

**Ask yourself these questions:**

1. **Have I read `.claude/WORKFLOW.md`?** (Required!)
2. **What workflow phase are we in?** (Spec review / Implementation / PR / Cleanup)
3. **What does the workflow say to do in this phase?**
4. **Do I need to ask the user for clarification?**
5. **Am I following the correct format for this phase?**

---

## Common Scenarios

### "Implement feature X"
→ Check: Are we past the specification review phase?
→ If yes: Follow Phase 2 (Implementation) in WORKFLOW.md
→ If no: Ask if we should do spec review first

### "Review @claude's feedback on issue #X"
→ This is Phase 1 (Specification & Review)
→ Follow: Read issue, read @claude's review, provide counter-review
→ Reference: WORKFLOW.md Phase 1

### "Address review feedback on PR #X"
→ This is Phase 3 (Pull Request)
→ Follow: Make changes, test, commit, push, post update
→ Reference: WORKFLOW.md Phase 3

### "The PR was merged"
→ This is Phase 4 (Cleanup)
→ Follow: Run cleanup-after-merge.sh
→ Reference: WORKFLOW.md Phase 4

---

## Testing Requirements

**Before EVERY commit:**
```bash
colcon test  # Must pass - no exceptions
```

**Coverage target:** 80%+ (where applicable)

**Test structure:**
- ROS 2 unit tests and integration tests
- Test navigation algorithms and safety features
- Test sensor data processing
- Verify message passing and transforms

---

## File References

**Primary Documentation (READ THESE):**
- `.claude/WORKFLOW.md` - **SINGLE SOURCE OF TRUTH** (read at start of every session!)
- `AUTOMATION-FAQ.md` - Automation details and Q&A
- `COMMENT-WRITING-GUIDE.md` - How to write good comments

**Configuration:**
- `.claude/settings.json` - ClaudeCode hooks configuration
- `.github/workflows/claude.yml` - @claude bot workflow (review-only)

**Scripts:**
- `scripts/start-work.sh` - Start work on issue
- `scripts/cleanup-after-merge.sh` - Post-merge cleanup
- `.claude/hooks/post-summary.sh` - Post progress comments

---

## Remember

**This file is just a pointer. The REAL workflow is in:**

```
.claude/WORKFLOW.md
```

**Read it at the start of EVERY session. Then ask yourself:**
1. What phase are we in?
2. What does the workflow say to do?
3. Am I following the correct process?

**When in doubt:**
1. Check `.claude/WORKFLOW.md`
2. Ask the user for clarification
3. Don't guess - follow the documented process

---

**Last Updated:** 2025-11-23
**Workflow Version:** See `.claude/WORKFLOW.md` for version history
**Maintained By:** This file ensures consistency across all ClaudeCode sessions
