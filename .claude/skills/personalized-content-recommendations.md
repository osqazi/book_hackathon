# Skill: Personalized Content Recommendations System

**Created**: 2025-12-18
**Context**: This skill encapsulates the knowledge and best practices for implementing a personalized content recommendation system based on user questionnaires. Based on the successful implementation in the Humanoid Robotics Book project.

## Overview

A complete personalization system that suggests relevant content based on user background and interests. This skill covers:
- User questionnaire system (minimal, focused questions)
- Intelligent recommendation engine (maps responses to actual content)
- Personalized UI components (card-based display)
- Integration with authentication system
- Database persistence for user preferences
- Production-ready deployment

## Key Principle

**ONLY recommend content that actually exists** - The system must map questionnaire responses to real, existing documentation pages to avoid 404 errors and maintain user trust.

## Architecture Pattern

### Three-Layer System

1. **Data Collection Layer** (Questionnaire)
   - Minimal questions (3-5 max) to reduce friction
   - Multiple choice for easy analysis
   - Questions map directly to content categories
   - Stored in user profile (via auth system)

2. **Intelligence Layer** (Recommendation Engine)
   - Analyzes user responses
   - Maps to actual content paths (verified to exist)
   - Priority-based scoring (1-5 scale)
   - Category classification (getting-started, fundamentals, practical, advanced)
   - Returns 6-10 most relevant recommendations

3. **Presentation Layer** (UI Components)
   - Card-based responsive layout
   - Priority badges for visual hierarchy
   - Reason display ("Matches your interest in...")
   - Empty state for users without questionnaire
   - Dark mode support

## File Structure

```
project-root/
├── src/
│   ├── lib/
│   │   ├── background-questions.ts        # Questionnaire definition
│   │   └── content-recommendations.ts     # Recommendation engine
│   ├── components/
│   │   ├── RecommendedForYou.tsx         # Main recommendation component
│   │   ├── RecommendedForYou.module.css  # Component styles
│   │   └── BackgroundQuestions.tsx        # Questionnaire UI
│   ├── pages/
│   │   └── personalization.tsx            # Personalization page
│   ├── hooks/
│   │   └── usePersonalization.ts          # API hooks for bookmarks
│   └── models/
│       └── user.ts                        # User data models
└── docs/                                  # Your actual content
    ├── intro.md
    ├── module-1/
    ├── module-2/
    └── ...
```

## Implementation Steps

### Step 1: Define Questionnaire (Keep It Minimal!)

**File**: `src/lib/background-questions.ts`

```typescript
export interface BackgroundQuestion {
  id: string;
  category: 'software' | 'hardware';
  questionText: string;
  options: string[];
  required: boolean;
}

export const backgroundQuestions: BackgroundQuestion[] = [
  {
    id: 'experience_level',
    category: 'software',
    questionText: 'What is your experience level?',
    options: ['Complete Beginner', 'Some Experience', 'Advanced'],
    required: false,
  },
  {
    id: 'programming_languages',
    category: 'software',
    questionText: 'Which programming languages do you know?',
    options: ['Python', 'JavaScript', 'Java', 'None yet'],
    required: false,
  },
  {
    id: 'primary_interest',
    category: 'software',
    questionText: 'Which topic interests you most?',
    options: [
      'Topic A (Fundamentals)',
      'Topic B (Advanced)',
      'Topic C (Practical)',
      'All topics equally'
    ],
    required: false,
  },
];
```

**Best Practices:**
- ✅ 3-5 questions maximum (reduces signup friction)
- ✅ Multiple choice only (easier to analyze)
- ✅ Options map directly to your content structure
- ✅ Make all optional (don't block signup)
- ❌ Avoid free text (hard to analyze)
- ❌ Don't ask more than necessary

### Step 2: Create Recommendation Engine

**File**: `src/lib/content-recommendations.ts`

```typescript
export interface RecommendedContent {
  title: string;
  path: string;  // Must match actual doc path!
  excerpt: string;
  reason: string;  // Why is this recommended?
  priority: number;  // 1-5, higher = more important
  category: 'getting-started' | 'fundamentals' | 'advanced' | 'practical';
}

export interface UserResponses {
  experience_level?: string[];
  programming_languages?: string[];
  primary_interest?: string[];
}

/**
 * Generate recommendations - ONLY returns real pages!
 */
export function getRecommendedContent(
  softwareBackground: any,
  hardwareBackground: any
): RecommendedContent[] {
  const recommendations: RecommendedContent[] = [];
  const responses: UserResponses = {
    ...softwareBackground,
    ...hardwareBackground
  };

  // Helper to check if user selected an option
  const hasResponse = (questionId: keyof UserResponses, value: string): boolean => {
    const response = responses[questionId];
    if (Array.isArray(response)) {
      return response.includes(value);
    }
    return response === value;
  };

  // Always recommend introduction
  recommendations.push({
    title: 'Introduction',
    path: '/docs/intro',  // Verify this path exists!
    excerpt: 'Start your learning journey here.',
    reason: 'Essential starting point',
    priority: 5,
    category: 'getting-started'
  });

  // Beginner path
  if (hasResponse('experience_level', 'Complete Beginner')) {
    recommendations.push({
      title: 'Getting Started Guide',
      path: '/docs/getting-started',  // Must be real path!
      excerpt: 'Learn the basics step by step.',
      reason: 'Perfect for beginners',
      priority: 5,
      category: 'getting-started'
    });
  }

  // Interest-based recommendations
  if (hasResponse('primary_interest', 'Topic A (Fundamentals)')) {
    recommendations.push({
      title: 'Topic A Overview',
      path: '/docs/topic-a',  // Verify path!
      excerpt: 'Deep dive into fundamental concepts.',
      reason: 'Matches your interest in fundamentals',
      priority: 5,
      category: 'fundamentals'
    });
  }

  // Python users
  if (hasResponse('programming_languages', 'Python')) {
    recommendations.push({
      title: 'Python Guide',
      path: '/docs/guides/python',  // Real path!
      excerpt: 'Python-specific tutorials and examples.',
      reason: 'Perfect for Python developers',
      priority: 4,
      category: 'practical'
    });
  }

  // Remove duplicates and sort by priority
  const uniqueRecommendations = Array.from(
    new Map(recommendations.map(item => [item.path, item])).values()
  );

  return uniqueRecommendations
    .sort((a, b) => b.priority - a.priority)
    .slice(0, 10);  // Top 10
}
```

**Critical Best Practices:**
- ✅ **VERIFY ALL PATHS EXIST** before adding to recommendations
- ✅ Use priority system (5=essential, 1=nice-to-have)
- ✅ Always include an introduction/getting-started page
- ✅ Remove duplicates before returning
- ✅ Limit to 10 recommendations (avoid overwhelming users)
- ❌ Never hardcode fictional paths
- ❌ Don't recommend the same page twice

### Step 3: Create UI Component

**File**: `src/components/RecommendedForYou.tsx`

```typescript
import React from 'react';
import Link from '@docusaurus/Link';
import { useAuthContext } from '@site/src/auth/context/AuthProvider';
import { getRecommendedContent } from '@site/src/lib/content-recommendations';
import styles from './RecommendedForYou.module.css';

const RecommendedForYou: React.FC = () => {
  const { session } = useAuthContext();

  const softwareBackground = session?.user?.softwareBackground || {};
  const hardwareBackground = session?.user?.hardwareBackground || {};

  const hasBackground =
    (softwareBackground && Object.keys(softwareBackground).length > 0) ||
    (hardwareBackground && Object.keys(hardwareBackground).length > 0);

  if (!hasBackground) {
    return (
      <div className={styles.emptyState}>
        <h2>Personalized Recommendations</h2>
        <p>Complete your profile to get personalized content recommendations!</p>
      </div>
    );
  }

  const recommendations = getRecommendedContent(
    softwareBackground,
    hardwareBackground
  );

  // Group by category
  const grouped = recommendations.reduce((acc, rec) => {
    if (!acc[rec.category]) acc[rec.category] = [];
    acc[rec.category].push(rec);
    return acc;
  }, {} as Record<string, typeof recommendations>);

  return (
    <div className={styles.recommendedSection}>
      <h2>📚 Personalized For You</h2>
      <p className={styles.subtitle}>
        Based on your questionnaire responses
      </p>

      {Object.entries(grouped).map(([category, items]) => (
        <div key={category} className={styles.categorySection}>
          <h3>{category.replace('-', ' ').toUpperCase()}</h3>
          <div className={styles.grid}>
            {items.map((rec, idx) => (
              <Link key={idx} to={rec.path} className={styles.card}>
                <h4>{rec.title}</h4>
                <span className={styles.badge}>Priority {rec.priority}</span>
                <p>{rec.excerpt}</p>
                <span className={styles.reason}>💡 {rec.reason}</span>
              </Link>
            ))}
          </div>
        </div>
      ))}
    </div>
  );
};

export default RecommendedForYou;
```

**File**: `src/components/RecommendedForYou.module.css`

```css
.recommendedSection {
  margin: 2rem 0;
  padding: 2rem;
  background: var(--ifm-background-surface-color);
  border-radius: 8px;
}

.grid {
  display: grid;
  grid-template-columns: repeat(auto-fill, minmax(300px, 1fr));
  gap: 1.5rem;
  margin-top: 1rem;
}

.card {
  display: block;
  padding: 1.5rem;
  background: var(--ifm-card-background-color, #fff);
  border: 1px solid var(--ifm-color-emphasis-200);
  border-radius: 8px;
  transition: all 0.3s ease;
  text-decoration: none;
  color: inherit;
}

.card:hover {
  border-color: var(--ifm-color-primary);
  box-shadow: 0 4px 12px rgba(0, 0, 0, 0.15);
  transform: translateY(-2px);
}

.badge {
  display: inline-block;
  padding: 0.25rem 0.5rem;
  background: #e3f2fd;
  color: #1976d2;
  border-radius: 4px;
  font-size: 0.75rem;
  font-weight: bold;
}

.reason {
  font-size: 0.85rem;
  color: var(--ifm-color-emphasis-600);
  font-style: italic;
}

.emptyState {
  text-align: center;
  padding: 3rem 2rem;
}
```

### Step 4: Integrate with Auth System

Ensure your auth system stores questionnaire responses in user profile:

```typescript
// In signup flow
const result = await authClient.signUp.email({
  email: data.email,
  password: data.password,
  softwareBackground: data.softwareBackground,  // From questionnaire
  hardwareBackground: data.hardwareBackground,
});
```

### Step 5: Add to Personalization Page

```typescript
// src/pages/personalization.tsx
import RecommendedForYou from '@site/src/components/RecommendedForYou';

const PersonalizationPage = () => {
  return (
    <Layout>
      <div className="container">
        {/* Recommendations section */}
        <RecommendedForYou />

        {/* Bookmarked chapters section */}
        <YourBookmarkedContent />
      </div>
    </Layout>
  );
};
```

## Content Path Verification

**Critical Step**: Before deploying, verify ALL recommendation paths exist!

```bash
# List all actual documentation files
find docs -type f -name "*.md" -o -name "*.mdx"

# Check a specific path
ls docs/intro.md  # Should exist

# Create a test script
node verify-recommendations.js
```

**File**: `verify-recommendations.js`

```javascript
import { getRecommendedContent } from './src/lib/content-recommendations.ts';
import fs from 'fs';
import path from 'path';

// Test with sample responses
const testResponses = [
  { experience_level: ['Complete Beginner'] },
  { primary_interest: ['Topic A (Fundamentals)'] },
  { programming_languages: ['Python'] },
];

testResponses.forEach((responses, idx) => {
  console.log(`\nTest Case ${idx + 1}:`, responses);
  const recommendations = getRecommendedContent(responses, {});

  recommendations.forEach(rec => {
    // Convert /docs/path to docs/path.md or docs/path/index.md
    const mdPath = `docs${rec.path.replace('/docs/', '/')}.md`;
    const mdxPath = `docs${rec.path.replace('/docs/', '/')}.mdx`;
    const indexPath = `docs${rec.path.replace('/docs/', '/')}/index.md`;

    const exists =
      fs.existsSync(mdPath) ||
      fs.existsSync(mdxPath) ||
      fs.existsSync(indexPath);

    console.log(
      exists ? '✅' : '❌',
      rec.path,
      '-',
      rec.title
    );

    if (!exists) {
      console.error(`ERROR: Path does not exist: ${rec.path}`);
      process.exit(1);
    }
  });
});

console.log('\n✅ All recommendation paths verified!');
```

## Production Deployment Checklist

- [ ] All recommendation paths verified to exist
- [ ] Questionnaire responses stored in database
- [ ] Environment detection working (localhost vs production)
- [ ] Empty state shows for users without questionnaire
- [ ] Recommendation count reasonable (6-10 items)
- [ ] No duplicate recommendations
- [ ] Priority badges display correctly
- [ ] Dark mode styling works
- [ ] Mobile responsive layout tested
- [ ] Links use correct domain in production

## Common Pitfalls and Solutions

### Pitfall 1: Broken Links (404 Errors)
**Problem**: Recommending content that doesn't exist
**Solution**:
- Always verify paths before adding to recommendations
- Use path verification script in CI/CD
- Start with fewer recommendations and expand gradually

### Pitfall 2: Too Many Questions
**Problem**: Users abandon signup due to long questionnaire
**Solution**:
- Keep to 3-5 questions maximum
- Make all questions optional
- Use multiple choice only (no free text)

### Pitfall 3: Generic Recommendations
**Problem**: All users see the same content
**Solution**:
- Implement diverse question responses
- Use priority system effectively
- Test with different user personas

### Pitfall 4: Stale Recommendations
**Problem**: Users see same recommendations every time
**Solution**:
- Consider adding "hide" functionality
- Rotate recommendations based on time
- Track what user has already read

### Pitfall 5: Localhost Hardcoding
**Problem**: API calls fail in production
**Solution**:
```typescript
const getBackendUrl = () => {
  if (typeof window !== 'undefined') {
    const hostname = window.location.hostname;
    if (hostname === 'yourdomain.com') {
      return 'https://api.yourdomain.com';
    }
  }
  return 'http://localhost:3001';
};
```

## Testing Strategy

### Unit Tests
```typescript
describe('getRecommendedContent', () => {
  it('returns introduction for all users', () => {
    const recs = getRecommendedContent({}, {});
    expect(recs.some(r => r.path === '/docs/intro')).toBe(true);
  });

  it('recommends beginner content for beginners', () => {
    const recs = getRecommendedContent(
      { experience_level: ['Complete Beginner'] },
      {}
    );
    expect(recs.some(r => r.category === 'getting-started')).toBe(true);
  });

  it('returns maximum 10 recommendations', () => {
    const recs = getRecommendedContent(
      { primary_interest: ['All topics equally'] },
      {}
    );
    expect(recs.length).toBeLessThanOrEqual(10);
  });
});
```

### Integration Tests
1. Sign up with different questionnaire responses
2. Visit personalization page
3. Verify correct recommendations appear
4. Click each recommended link
5. Confirm no 404 errors

### User Acceptance Testing
- [ ] Complete beginner sees appropriate starter content
- [ ] Advanced user sees advanced topics
- [ ] Python developer sees Python-specific content
- [ ] User with multiple interests sees diverse recommendations
- [ ] Empty state shows for users without questionnaire

## Performance Considerations

- **Client-Side Computation**: Recommendations computed in browser (fast, no API call)
- **Caching**: Results stable within session (no recalculation on navigation)
- **Lazy Loading**: Component only renders when personalization page visited
- **Small Bundle**: Recommendation logic is lightweight (<5KB)

## Analytics and Iteration

Track these metrics to improve recommendations:

```typescript
// Track when user clicks a recommendation
const trackRecommendationClick = (recommendationPath: string, reason: string) => {
  analytics.track('recommendation_clicked', {
    path: recommendationPath,
    reason: reason,
    timestamp: new Date().toISOString()
  });
};
```

**Metrics to monitor:**
- Click-through rate per recommendation
- Most popular content categories
- Questionnaire completion rate
- Time to first content interaction
- User retention after personalization

## Future Enhancements

1. **Machine Learning**: Train on actual user behavior
2. **Completion Tracking**: Recommend based on what user finished
3. **Difficulty Progression**: Suggest logical next chapter
4. **Collaborative Filtering**: "Users like you also read..."
5. **Time Estimates**: Show expected reading time
6. **Learning Paths**: Create complete curriculum based on goals

## Example Implementation Timeline

**Day 1**: Questionnaire design and user model updates
**Day 2**: Recommendation engine with path verification
**Day 3**: UI components and styling
**Day 4**: Integration and testing
**Day 5**: Production deployment and monitoring

## Real-World Results

From Humanoid Robotics Book implementation:
- ✅ Reduced questionnaire from 6 to 3 questions (50% reduction)
- ✅ 100% working links (0% 404 rate)
- ✅ 6-10 personalized recommendations per user
- ✅ Clear user feedback: "Thats 100% perfect now"
- ✅ No performance impact (client-side computation)

## References

- Project implementation: `humanoid-robotics-book`
- Auth integration: `.claude/skills/better-auth-integration.md`
- PHR record: `history/prompts/general/0017-fixed-production-auth-and-personalization-system.general.prompt.md`

---

**Key Takeaway**: Keep it simple, verify all paths, and iterate based on user behavior. A working system with 5 recommendations is better than a broken system with 50.
