// validate-auth-implementation.js
// Quick validation script to verify the Better-Auth implementation

const fs = require('fs');
const path = require('path');

// Define the expected files and directories
const expectedFiles = [
  // Core auth configuration
  'src/lib/better-auth/auth.ts',

  // Database configuration
  'src/lib/database/index.ts',
  'src/lib/database/schema.ts',

  // User models
  'src/models/user.ts',

  // Auth context
  'src/auth/context/AuthProvider.tsx',

  // Auth hooks
  'src/auth/hooks/useAuth.ts',

  // Auth services
  'src/auth/services/auth-service.ts',

  // Auth components
  'src/auth/components/SignupForm.tsx',
  'src/auth/components/BackgroundQuestions.tsx',
  'src/auth/components/SigninForm.tsx',

  // Personalization components
  'src/components/personalization/ContentAdapter.tsx',
  'src/components/personalization/ExampleAdapter.tsx',

  // Utils
  'src/auth/utils/session-utils.ts',

  // Background questions
  'src/lib/background-questions.ts',

  // Schema
  'schema/auth.sql',

  // Auth server
  'auth-server/server.js',
  'auth-server/package.json',

  // Docusaurus plugin
  'plugins/docusaurus-auth/index.js',
  'plugins/docusaurus-auth/src/AuthProviderWrapper.js',

  // Documentation
  'docs/auth-setup.md',
];

console.log('🔍 Validating Better-Auth Implementation...\n');

let successCount = 0;
let totalCount = expectedFiles.length;

for (const file of expectedFiles) {
  const filePath = path.join(__dirname, file);
  const exists = fs.existsSync(filePath);

  if (exists) {
    console.log(`✅ ${file}`);
    successCount++;
  } else {
    console.log(`❌ ${file}`);
  }
}

console.log(`\n📊 Validation Summary: ${successCount}/${totalCount} files present`);

// Check package.json for required dependencies
const packageJson = JSON.parse(fs.readFileSync(path.join(__dirname, 'package.json'), 'utf8'));
const requiredDeps = ['better-auth', 'drizzle-orm', 'drizzle-kit', 'better-sqlite3'];
const missingDeps = [];

for (const dep of requiredDeps) {
  if (!packageJson.dependencies[dep] && !packageJson.devDependencies[dep]) {
    missingDeps.push(dep);
  }
}

if (missingDeps.length > 0) {
  console.log(`❌ Missing dependencies: ${missingDeps.join(', ')}`);
} else {
  console.log(`✅ All required dependencies present`);
}

// Check docusaurus.config.ts for plugin inclusion
const docusaurusConfig = fs.readFileSync(path.join(__dirname, 'docusaurus.config.ts'), 'utf8');
if (docusaurusConfig.includes('./plugins/docusaurus-auth')) {
  console.log(`✅ Docusaurus plugin properly configured`);
} else {
  console.log(`❌ Docusaurus plugin not found in config`);
}

console.log('\n🎉 Validation complete!');
console.log(`Overall status: ${successCount === totalCount && missingDeps.length === 0 ? '✅ PASS' : '❌ FAIL'}`);