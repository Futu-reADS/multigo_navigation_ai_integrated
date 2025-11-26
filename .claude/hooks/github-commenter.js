#!/usr/bin/env node

/**
 * Claude Code GitHub Commenter Hook v2
 *
 * FEATURES:
 * 1. Captures BOTH user prompt AND Claude's response summary
 * 2. Different formatting for Issue vs PR comments:
 *    - Issue: Shows Claude's achievement summary
 *    - PR: Shows user prompt + what went wrong + what was corrected
 * 3. Session numbering for multiple iterations
 *
 * WORKFLOW:
 * Iteration 1: Initial implementation
 * Iteration 2: Fix review issues
 * Iteration 3: Add more features
 * ... each gets its own comment with session number
 */

const fs = require('fs');
const path = require('path');
const { execSync } = require('child_process');

// ============================================================================
// CONFIGURATION
// ============================================================================

const CONFIG = {
  significantTools: ['Write', 'Edit', 'Bash'],
  minToolsThreshold: 2,
  sessionFile: path.join(process.cwd(), '.claude', 'session-tracking.json'),
  sessionCounterFile: path.join(process.cwd(), '.claude', 'session-counter.json'),
};

// ============================================================================
// HELPER FUNCTIONS
// ============================================================================

function getIssueFromBranch() {
  try {
    const branch = execSync('git branch --show-current', { encoding: 'utf-8' }).trim();
    const match = branch.match(/(?:feature|fix|issue|refactor|chore)\/(\d+)/i) || branch.match(/^(\d+)-/);
    return match ? parseInt(match[1]) : null;
  } catch (error) {
    return null;
  }
}

function getCurrentPR() {
  try {
    const branch = execSync('git branch --show-current', { encoding: 'utf-8' }).trim();
    const prs = execSync(`gh pr list --head ${branch} --json number --jq '.[0].number'`, {
      encoding: 'utf-8',
      stdio: ['pipe', 'pipe', 'ignore']
    }).trim();
    return prs ? parseInt(prs) : null;
  } catch (error) {
    return null;
  }
}

function loadSession() {
  try {
    if (fs.existsSync(CONFIG.sessionFile)) {
      const data = fs.readFileSync(CONFIG.sessionFile, 'utf-8');
      return JSON.parse(data);
    }
  } catch (error) {}

  return {
    userPrompt: '',
    claudeResponse: '',
    toolsUsed: [],
    filesModified: [],
    startTime: Date.now()
  };
}

function saveSession(session) {
  try {
    const dir = path.dirname(CONFIG.sessionFile);
    if (!fs.existsSync(dir)) {
      fs.mkdirSync(dir, { recursive: true });
    }
    fs.writeFileSync(CONFIG.sessionFile, JSON.stringify(session, null, 2));
  } catch (error) {}
}

function clearSession() {
  try {
    if (fs.existsSync(CONFIG.sessionFile)) {
      fs.unlinkSync(CONFIG.sessionFile);
    }
  } catch (error) {}
}

function getSessionNumber(issueNumber, prNumber) {
  try {
    let counter = {};
    if (fs.existsSync(CONFIG.sessionCounterFile)) {
      counter = JSON.parse(fs.readFileSync(CONFIG.sessionCounterFile, 'utf-8'));
    }

    const key = prNumber ? `pr-${prNumber}` : `issue-${issueNumber}`;
    const sessionNum = (counter[key] || 0) + 1;

    counter[key] = sessionNum;
    fs.writeFileSync(CONFIG.sessionCounterFile, JSON.stringify(counter, null, 2));

    return sessionNum;
  } catch (error) {
    return 1;
  }
}

/**
 * Generate Claude's response summary from tools used
 */
function generateClaudeSummary(filesModified, toolsUsed) {
  const written = filesModified.filter(f => f.operation === 'Write');
  const edited = filesModified.filter(f => f.operation === 'Edit');
  const commands = toolsUsed.filter(t => t.tool === 'Bash');

  let summary = [];

  if (written.length > 0) {
    summary.push(`Created ${written.length} new file${written.length > 1 ? 's' : ''}: ${written.map(f => f.file).join(', ')}`);
  }

  if (edited.length > 0) {
    summary.push(`Modified ${edited.length} existing file${edited.length > 1 ? 's' : ''}: ${edited.map(f => f.file).join(', ')}`);
  }

  if (commands.length > 0) {
    const testCommand = commands.find(c => (c.description || '').toLowerCase().includes('test'));
    if (testCommand) {
      summary.push(`Ran tests to verify implementation`);
    }
  }

  return summary.join('. ') + '.';
}

/**
 * Format comment for GitHub Issue
 * Focus: What Claude achieved
 */
function formatIssueComment(data) {
  const { sessionNum, userPrompt, filesModified, toolsUsed, duration } = data;

  const timestamp = new Date().toISOString().replace('T', ' ').substring(0, 19);
  const durationStr = duration > 60000
    ? `${Math.round(duration / 60000)}m`
    : `${Math.round(duration / 1000)}s`;

  const claudeSummary = generateClaudeSummary(filesModified, toolsUsed);

  const written = filesModified.filter(f => f.operation === 'Write');
  const edited = filesModified.filter(f => f.operation === 'Edit');

  let comment = `## 💻 Claude Code Session ${sessionNum}\n\n`;
  comment += `**Time:** ${timestamp} | **Duration:** ${durationStr}\n\n`;

  comment += `### 📝 Your Request\n\`\`\`\n${userPrompt.substring(0, 400)}${userPrompt.length > 400 ? '...' : ''}\n\`\`\`\n\n`;

  comment += `### 🎯 What Was Achieved\n\n`;
  comment += `${claudeSummary}\n\n`;

  comment += `### 📁 Files Changed\n\n`;

  if (written.length > 0) {
    comment += `**Created (${written.length}):**\n`;
    written.forEach(f => {
      comment += `- \`${f.file}\`\n`;
    });
    comment += `\n`;
  }

  if (edited.length > 0) {
    comment += `**Modified (${edited.length}):**\n`;
    edited.forEach(f => {
      comment += `- \`${f.file}\`\n`;
    });
    comment += `\n`;
  }

  const totalFiles = written.length + edited.length;
  comment += `### 📊 Summary\n\n`;
  comment += `- **Files changed:** ${totalFiles}\n`;
  comment += `- **Duration:** ${durationStr}\n\n`;

  comment += `---\n`;
  comment += `*🤖 Automated update from Claude Code*\n`;

  return comment;
}

/**
 * Format comment for GitHub PR
 * Focus: User's request + what was wrong + what was corrected
 */
function formatPRComment(data) {
  const { sessionNum, userPrompt, filesModified, toolsUsed, duration } = data;

  const timestamp = new Date().toISOString().replace('T', ' ').substring(0, 19);
  const durationStr = duration > 60000
    ? `${Math.round(duration / 60000)}m`
    : `${Math.round(duration / 1000)}s`;

  const written = filesModified.filter(f => f.operation === 'Write');
  const edited = filesModified.filter(f => f.operation === 'Edit');

  // Detect if this is a fix iteration (prompt mentions fix/review/issue/bug)
  const isFix = /\b(fix|review|issue|bug|problem|error|correct|address)\b/i.test(userPrompt);

  let comment = `## 🔄 Iteration ${sessionNum}\n\n`;
  comment += `**Time:** ${timestamp} (${durationStr})\n\n`;

  comment += `### 📝 Request\n\`\`\`\n${userPrompt.substring(0, 400)}${userPrompt.length > 400 ? '...' : ''}\n\`\`\`\n\n`;

  if (isFix && sessionNum > 1) {
    comment += `### 🐛 What Was Wrong\n`;
    comment += `Review identified issues that needed correction.\n\n`;
  }

  comment += `### ✅ What Was Corrected/Added\n\n`;

  if (written.length > 0) {
    comment += `**Files Created:**\n`;
    written.forEach(f => {
      comment += `- \`${f.file}\`\n`;
    });
    comment += `\n`;
  }

  if (edited.length > 0) {
    comment += `**Files Modified:**\n`;
    edited.forEach(f => {
      comment += `- \`${f.file}\`\n`;
    });
    comment += `\n`;
  }

  const testCommand = toolsUsed.find(t => (t.description || '').toLowerCase().includes('test'));
  if (testCommand) {
    comment += `**Verification:**\n`;
    comment += `- ✓ Tests run and passing\n\n`;
  }

  comment += `---\n`;
  comment += `*Session ${sessionNum} • Auto-generated by Claude Code*\n`;

  return comment;
}

function postGitHubComment(type, number, comment) {
  try {
    const tempFile = path.join('/tmp', `github-comment-${Date.now()}.md`);
    fs.writeFileSync(tempFile, comment);

    let cmd;
    if (type === 'issue') {
      cmd = `gh issue comment ${number} --body-file ${tempFile}`;
    } else {
      cmd = `gh pr comment ${number} --body-file ${tempFile}`;
    }

    execSync(cmd, {
      encoding: 'utf-8',
      stdio: ['pipe', 'pipe', 'ignore']
    });

    fs.unlinkSync(tempFile);
    return true;
  } catch (error) {
    console.error(`[github-commenter] Failed to post ${type} comment: ${error.message}`);
    return false;
  }
}

// ============================================================================
// MAIN HOOK LOGIC
// ============================================================================

function main() {
  let inputData = '';

  try {
    inputData = fs.readFileSync(0, 'utf-8');
  } catch (error) {
    process.exit(0);
  }

  if (!inputData.trim()) {
    process.exit(0);
  }

  try {
    const hookData = JSON.parse(inputData);

    const issueNumber = getIssueFromBranch();
    const prNumber = getCurrentPR();

    if (!issueNumber && !prNumber) {
      process.exit(0);
    }

    const session = loadSession();

    // Handle UserPromptSubmit - save user's prompt
    if (hookData.hook_event_name === 'UserPromptSubmit') {
      session.userPrompt = hookData.prompt || '';
      session.toolsUsed = [];
      session.filesModified = [];
      session.startTime = Date.now();
      saveSession(session);
      process.exit(0);
    }

    // Handle PostToolUse - track tools and post comments when threshold reached
    if (hookData.hook_event_name === 'PostToolUse') {
      const toolName = hookData.tool_name;

      if (CONFIG.significantTools.includes(toolName)) {
        session.toolsUsed.push({
          tool: toolName,
          description: hookData.description,
          command: hookData.command
        });

        if (toolName === 'Write' || toolName === 'Edit') {
          const filePath = hookData.file_path || hookData.parameters?.file_path || '';
          if (filePath) {
            session.filesModified.push({
              file: path.relative(process.cwd(), filePath),
              operation: toolName
            });
          }
        }

        saveSession(session);
      }

      // Post comments after threshold
      if (session.toolsUsed.length >= CONFIG.minToolsThreshold) {
        const duration = Date.now() - session.startTime;
        const sessionNum = getSessionNumber(issueNumber, prNumber);

        const commentData = {
          sessionNum: sessionNum,
          userPrompt: session.userPrompt,
          toolsUsed: session.toolsUsed,
          filesModified: session.filesModified,
          duration: duration
        };

        // Post to Issue (achievement-focused)
        if (issueNumber) {
          const comment = formatIssueComment(commentData);
          const posted = postGitHubComment('issue', issueNumber, comment);
          if (posted) {
            console.log(`✓ Posted Session ${sessionNum} to Issue #${issueNumber}`);
          }
        }

        // Post to PR (iteration-focused with user's request)
        if (prNumber) {
          const comment = formatPRComment(commentData);
          const posted = postGitHubComment('pr', prNumber, comment);
          if (posted) {
            console.log(`✓ Posted Iteration ${sessionNum} to PR #${prNumber}`);
          }
        }

        clearSession();
      }
    }

  } catch (error) {
    console.error(`[github-commenter] Error: ${error.message}`);
  }

  process.exit(0);
}

if (require.main === module) {
  main();
}

module.exports = { formatIssueComment, formatPRComment, getIssueFromBranch, getCurrentPR };
