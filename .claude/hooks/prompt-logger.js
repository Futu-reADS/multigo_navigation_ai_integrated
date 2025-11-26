#!/usr/bin/env node

/**
 * Claude Code Prompt Logger Hook
 *
 * Automatically logs important interactions between user and Claude Code
 * Filters out trivial conversations and saves only development-relevant context
 *
 * Usage: This hook runs after each Claude Code interaction
 */

const fs = require('fs');
const path = require('path');
const { execSync } = require('child_process');

// ============================================================================
// CONFIGURATION
// ============================================================================

const CONFIG = {
  // Where to save development logs
  logsDir: path.join(process.cwd(), 'docs', 'dev-logs'),

  // Minimum prompt length to consider logging (filters out trivial questions)
  minPromptLength: 50,

  // Keywords that indicate important technical conversations
  technicalKeywords: [
    'implement', 'create', 'add', 'fix', 'bug', 'error', 'test',
    'refactor', 'optimize', 'deploy', 'build', 'issue', 'feature',
    'architecture', 'design', 'database', 'api', 'endpoint',
    'function', 'class', 'component', 'service', 'model'
  ],

  // Keywords to ignore (casual conversations)
  ignoreKeywords: [
    'hello', 'hi', 'thanks', 'thank you', 'bye',
    'what time', 'what date', 'weather'
  ],

  // File patterns to ignore when logging file operations
  ignoreFilePatterns: [
    /node_modules/,
    /\.git\//,
    /dist\//,
    /build\//,
    /coverage\//
  ]
};

// ============================================================================
// HELPER FUNCTIONS
// ============================================================================

/**
 * Check if conversation is technically relevant
 */
function isRelevantConversation(userPrompt, claudeResponse) {
  const combined = (userPrompt + ' ' + (claudeResponse || '')).toLowerCase();

  // Too short = probably trivial
  if (userPrompt.length < CONFIG.minPromptLength) {
    return false;
  }

  // Contains ignore keywords = casual chat
  const hasIgnoreKeyword = CONFIG.ignoreKeywords.some(kw =>
    combined.includes(kw.toLowerCase())
  );
  if (hasIgnoreKeyword && !hasTechnicalKeyword(combined)) {
    return false;
  }

  // Contains technical keywords = important
  if (hasTechnicalKeyword(combined)) {
    return true;
  }

  // For UserPromptSubmit (no response yet), check prompt only
  if (!claudeResponse) {
    // Log if it contains file extensions
    if (userPrompt.match(/\b\w+\.(js|py|ts|jsx|tsx|java|go|rs|cpp|h|css|html|json|yaml|yml|md)\b/)) {
      return true;
    }
    // Log if it mentions actions
    if (userPrompt.match(/\b(write|read|create|update|delete|modify|change|add|remove|fix|debug|test|deploy)\b/i)) {
      return true;
    }
  }

  // Contains code blocks = important (only if response exists)
  if (claudeResponse && (claudeResponse.includes('```') || claudeResponse.includes('`'))) {
    return true;
  }

  // Modified files mentioned = important
  if (combined.match(/\b\w+\.(js|py|ts|jsx|tsx|java|go|rs|cpp|h|css|html)\b/)) {
    return true;
  }

  return false;
}

function hasTechnicalKeyword(text) {
  return CONFIG.technicalKeywords.some(kw =>
    text.toLowerCase().includes(kw.toLowerCase())
  );
}

/**
 * Extract current git branch and issue number
 */
function getGitContext() {
  try {
    const branch = execSync('git branch --show-current', { encoding: 'utf-8' }).trim();

    // Extract issue number from branch name (e.g., feature/45-user-auth → 45)
    const issueMatch = branch.match(/(\d+)/);
    const issueNumber = issueMatch ? issueMatch[1] : null;

    return { branch, issueNumber };
  } catch (error) {
    return { branch: 'unknown', issueNumber: null };
  }
}

/**
 * Get list of modified files in current branch
 */
function getModifiedFiles() {
  try {
    const files = execSync('git diff --name-only HEAD', { encoding: 'utf-8' })
      .split('\n')
      .filter(f => f.trim() && !CONFIG.ignoreFilePatterns.some(p => p.test(f)));

    return files;
  } catch (error) {
    return [];
  }
}

/**
 * Format log entry
 */
function formatLogEntry(data) {
  const { userPrompt, claudeResponse, gitContext, modifiedFiles, timestamp } = data;

  let entry = `
## Session: ${timestamp}
**Branch:** \`${gitContext.branch}\`
${gitContext.issueNumber ? `**Issue:** #${gitContext.issueNumber}` : ''}

### 📝 User Prompt
\`\`\`
${userPrompt.trim()}
\`\`\`
`;

  // Only add response section if we have a response
  if (claudeResponse) {
    entry += `
### 🤖 Claude Response Summary
${extractSummary(claudeResponse)}
`;
  }

  // Add modified files if any
  if (modifiedFiles.length > 0) {
    entry += `
### 📁 Files Modified
${modifiedFiles.map(f => `- ${f}`).join('\n')}
`;
  }

  entry += `
---
`;

  return entry;
}

/**
 * Extract key points from Claude's response (first 500 chars + code blocks)
 */
function extractSummary(response) {
  // Get first paragraph or 500 chars
  let summary = response.substring(0, 500);

  // If there are code blocks, include them
  const codeBlocks = response.match(/```[\s\S]*?```/g);
  if (codeBlocks && codeBlocks.length > 0) {
    summary += '\n\n**Code snippets provided:**\n' + codeBlocks[0];
  }

  if (response.length > 500) {
    summary += '\n\n_[Response truncated for brevity. See full context in Claude Code history.]_';
  }

  return summary;
}

/**
 * Save prompt to .claude/prompt-history.json for auto-summary integration
 */
function savePromptHistory(userPrompt, gitContext, timestamp) {
  const claudeDir = path.join(process.cwd(), '.claude');
  const historyFile = path.join(claudeDir, 'prompt-history.json');

  // Ensure .claude directory exists
  if (!fs.existsSync(claudeDir)) {
    fs.mkdirSync(claudeDir, { recursive: true });
  }

  // Read existing history
  let history = [];
  if (fs.existsSync(historyFile)) {
    try {
      const content = fs.readFileSync(historyFile, 'utf-8');
      history = JSON.parse(content);
      if (!Array.isArray(history)) {
        history = [];
      }
    } catch (error) {
      // Invalid JSON, start fresh
      history = [];
    }
  }

  // Add new prompt entry
  history.push({
    prompt: userPrompt,
    timestamp: timestamp,
    issueNumber: gitContext.issueNumber || null,
    branch: gitContext.branch
  });

  // Keep only last 50 prompts to prevent file bloat
  if (history.length > 50) {
    history = history.slice(-50);
  }

  // Save updated history
  fs.writeFileSync(historyFile, JSON.stringify(history, null, 2));
}

/**
 * Save log entry to appropriate file
 */
function saveLogEntry(entry, gitContext) {
  // Ensure logs directory exists
  if (!fs.existsSync(CONFIG.logsDir)) {
    fs.mkdirSync(CONFIG.logsDir, { recursive: true });
  }

  // Determine log file name
  let logFileName;
  if (gitContext.issueNumber) {
    logFileName = `issue-${gitContext.issueNumber}.md`;
  } else {
    const date = new Date().toISOString().split('T')[0];
    logFileName = `session-${date}.md`;
  }

  const logFilePath = path.join(CONFIG.logsDir, logFileName);

  // Create or append to log file
  if (!fs.existsSync(logFilePath)) {
    const header = `# Development Log${gitContext.issueNumber ? ` - Issue #${gitContext.issueNumber}` : ''}

_Auto-generated by Claude Code Prompt Logger_

---
`;
    fs.writeFileSync(logFilePath, header);
  }

  fs.appendFileSync(logFilePath, entry);

  return logFilePath;
}

/**
 * Update index file
 */
function updateIndex(logFilePath, gitContext) {
  const indexPath = path.join(CONFIG.logsDir, 'README.md');

  if (!fs.existsSync(indexPath)) {
    fs.writeFileSync(indexPath, `# Development Logs Index

This directory contains auto-generated development logs from Claude Code sessions.

## Recent Sessions

`);
  }

  const logFileName = path.basename(logFilePath);
  const timestamp = new Date().toISOString();
  const indexEntry = `- [${timestamp}] [${logFileName}](./${logFileName}) - Branch: \`${gitContext.branch}\`\n`;

  // Read current index
  let indexContent = fs.readFileSync(indexPath, 'utf-8');

  // Add entry if not already present
  if (!indexContent.includes(logFileName)) {
    indexContent += indexEntry;
    fs.writeFileSync(indexPath, indexContent);
  }
}

// ============================================================================
// MAIN HOOK LOGIC
// ============================================================================

function main() {
  // Read input from stdin (Claude Code passes conversation data as JSON)
  let inputData = '';

  // Check if there's stdin input (synchronous read)
  try {
    inputData = fs.readFileSync(0, 'utf-8');
  } catch (error) {
    console.error('No input provided to hook');
    process.exit(0);
  }

  if (!inputData.trim()) {
    console.error('Empty input provided to hook');
    process.exit(0);
  }

  try {
    // Parse hook input JSON
    const hookData = JSON.parse(inputData);

    // Extract prompt from UserPromptSubmit hook data
    const userPrompt = hookData.prompt || '';

    // For UserPromptSubmit, we don't have Claude's response yet
    // So we'll just log the prompt
    const claudeResponse = '';

    // Log the prompt we received (for debugging)
    // console.error(`[DEBUG] Received prompt: ${userPrompt.substring(0, 100)}...`);

    // Filter: Only log relevant conversations
    if (!isRelevantConversation(userPrompt, claudeResponse)) {
      // Silent skip - don't pollute output
      process.exit(0);
    }

    // Gather context
    const gitContext = getGitContext();
    const modifiedFiles = getModifiedFiles();
    const timestamp = new Date().toISOString().replace('T', ' ').substring(0, 19);

    // Format log entry
    const entry = formatLogEntry({
      userPrompt,
      claudeResponse,
      gitContext,
      modifiedFiles,
      timestamp
    });

    // Save to prompt history (for auto-summary integration)
    savePromptHistory(userPrompt, gitContext, timestamp);

    // Save to file
    const logFilePath = saveLogEntry(entry, gitContext);
    updateIndex(logFilePath, gitContext);

    // Output success message (will be shown in Claude Code)
    console.log(`✓ Prompt logged to ${path.basename(logFilePath)}`);

  } catch (error) {
    // Log error to stderr (won't block Claude Code)
    console.error(`[prompt-logger] Error: ${error.message}`);
    // Always exit 0 to not fail the main Claude Code operation
    process.exit(0);
  }
}

// Run hook
if (require.main === module) {
  main();
}

module.exports = { isRelevantConversation, formatLogEntry };
