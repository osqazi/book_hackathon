/**
 * Content Validation Script for Humanoid Robotics Book
 * Validates module word counts, code examples, diagrams, and citations
 */

const fs = require('fs');
const path = require('path');

/**
 * Count words in markdown content (excluding code blocks and frontmatter)
 */
function countWords(content) {
  // Remove frontmatter
  content = content.replace(/^---[\s\S]*?---/m, '');

  // Remove code blocks
  content = content.replace(/```[\s\S]*?```/g, '');

  // Remove inline code
  content = content.replace(/`[^`]+`/g, '');

  // Count words
  const words = content.trim().split(/\s+/).filter(word => word.length > 0);
  return words.length;
}

/**
 * Count code examples in markdown content
 */
function countCodeExamples(content) {
  const codeBlocks = content.match(/```[\s\S]*?```/g);
  return codeBlocks ? codeBlocks.length : 0;
}

/**
 * Count Mermaid diagrams in markdown content
 */
function countDiagrams(content) {
  const mermaidBlocks = content.match(/```mermaid[\s\S]*?```/g);
  return mermaidBlocks ? mermaidBlocks.length : 0;
}

/**
 * Count citations (footnote references) in markdown content
 */
function countCitations(content) {
  const citations = content.match(/\[\^[^\]]+\]/g);
  return citations ? citations.length : 0;
}

/**
 * Validate a module directory
 */
function validateModule(modulePath, minWords, maxWords, minExamples, minDiagrams) {
  const results = {
    modulePath,
    wordCount: 0,
    codeExamples: 0,
    diagrams: 0,
    citations: 0,
    files: [],
    passed: false,
  };

  try {
    // Read all markdown files in module directory
    const files = fs.readdirSync(modulePath).filter(f => f.endsWith('.md'));

    files.forEach(file => {
      const filePath = path.join(modulePath, file);
      const content = fs.readFileSync(filePath, 'utf-8');

      const fileWords = countWords(content);
      const fileExamples = countCodeExamples(content);
      const fileDiagrams = countDiagrams(content);
      const fileCitations = countCitations(content);

      results.wordCount += fileWords;
      results.codeExamples += fileExamples;
      results.diagrams += fileDiagrams;
      results.citations += fileCitations;

      results.files.push({
        name: file,
        words: fileWords,
        examples: fileExamples,
        diagrams: fileDiagrams,
        citations: fileCitations,
      });
    });

    // Validate against requirements
    results.passed =
      results.wordCount >= minWords &&
      results.wordCount <= maxWords &&
      results.codeExamples >= minExamples &&
      results.diagrams >= minDiagrams;

  } catch (error) {
    console.error('Error validating module ' + modulePath + ':', error.message);
  }

  return results;
}

/**
 * Main validation function
 */
function main() {
  console.log('=== Humanoid Robotics Book - Content Validation ===');
  console.log('');

  const modules = [
    {
      name: 'Module 1: ROS 2 Fundamentals',
      path: path.join(__dirname, '..', 'docs', 'module-1-ros2'),
      minWords: 4000,
      maxWords: 6000,
      minExamples: 4,
      minDiagrams: 1,
    },
    {
      name: 'Module 2: Simulation & Digital Twins',
      path: path.join(__dirname, '..', 'docs', 'module-2-simulation'),
      minWords: 4500,
      maxWords: 7000,
      minExamples: 0,
      minDiagrams: 3,
    },
    {
      name: 'Module 3: AI-Driven Perception',
      path: path.join(__dirname, '..', 'docs', 'module-3-isaac'),
      minWords: 5000,
      maxWords: 7000,
      minExamples: 0,
      minDiagrams: 3,
    },
    {
      name: 'Module 4: Vision-Language-Action',
      path: path.join(__dirname, '..', 'docs', 'module-4-vla'),
      minWords: 5000,
      maxWords: 8000,
      minExamples: 3,
      minDiagrams: 1,
    },
  ];

  let allPassed = true;

  modules.forEach(function(module) {
    const results = validateModule(
      module.path,
      module.minWords,
      module.maxWords,
      module.minExamples,
      module.minDiagrams
    );

    console.log(module.name + ':');
    console.log('  Path: ' + module.path);
    console.log('  Word Count: ' + results.wordCount + ' (target: ' + module.minWords + '-' + module.maxWords + ')');
    console.log('  Code Examples: ' + results.codeExamples + ' (min: ' + module.minExamples + ')');
    console.log('  Diagrams: ' + results.diagrams + ' (min: ' + module.minDiagrams + ')');
    console.log('  Citations: ' + results.citations);
    console.log('  Status: ' + (results.passed ? ' PASS' : 'L FAIL'));
    console.log('');

    if (!results.passed) {
      allPassed = false;
    }
  });

  console.log('Overall Status: ' + (allPassed ? ' ALL MODULES PASS' : 'L SOME MODULES FAIL'));
  process.exit(allPassed ? 0 : 1);
}

// Export functions for testing
module.exports = {
  validateModule: validateModule,
  countWords: countWords,
  countCodeExamples: countCodeExamples,
  countDiagrams: countDiagrams,
  countCitations: countCitations,
};

// Run main if executed directly
if (require.main === module) {
  main();
}
