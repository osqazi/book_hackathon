import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import styles from './index.module.css';

function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header className={clsx('hero hero--primary', styles.heroBanner)}>
      <div className="container">
        <h1 className={styles.heroTitle}>{siteConfig.title}</h1>
        <p className={styles.heroSubtitle}>{siteConfig.tagline}</p>
        <p className={styles.heroDescription}>
          A comprehensive guide to building intelligent humanoid robots using modern robotics frameworks and AI systems.
        </p>
        <div className={styles.buttons}>
          <Link
            className="button button--secondary button--lg"
            to="/docs/intro">
            Start Reading
          </Link>
          <Link
            className={clsx('button button--outline button--secondary button--lg', styles.buttonGithub)}
            to="https://github.com/osqazi/book_hackathon">
            View on GitHub
          </Link>
        </div>
      </div>
    </header>
  );
}

function BookFeatures() {
  return (
    <section className={styles.features}>
      <div className="container">
        <h2 className={styles.sectionTitle}>What You'll Learn</h2>
        <div className={styles.featureGrid}>
          <div className={styles.featureCard}>
            <div className={styles.featureIcon}>🤖</div>
            <h3>ROS 2 Fundamentals</h3>
            <p>Master Robot Operating System 2, the industry-standard framework for robotics development.</p>
          </div>
          <div className={styles.featureCard}>
            <div className={styles.featureIcon}>🎮</div>
            <h3>Simulation & Testing</h3>
            <p>Learn to simulate and test humanoid robots in virtual environments using Gazebo and Isaac Sim.</p>
          </div>
          <div className={styles.featureCard}>
            <div className={styles.featureIcon}>⚡</div>
            <h3>NVIDIA Isaac Platform</h3>
            <p>Leverage GPU-accelerated robotics simulation and AI training with NVIDIA's Isaac platform.</p>
          </div>
          <div className={styles.featureCard}>
            <div className={styles.featureIcon}>👁️</div>
            <h3>Vision-Language-Action</h3>
            <p>Integrate cutting-edge VLA models to enable robots to understand and act on natural language commands.</p>
          </div>
        </div>
      </div>
    </section>
  );
}

function AuthorSection() {
  return (
    <section className={styles.authorSection}>
      <div className="container">
        <div className={styles.authorCard}>
          <div className={styles.authorContent}>
            <h2 className={styles.authorTitle}>About the Author</h2>
            <h3 className={styles.authorName}>Owais Qazi</h3>
            <p className={styles.authorRole}>Founder & CEO, MetaLog Inc.</p>
            <p className={styles.authorBio}>
              Owais Qazi is a leading expert in robotics and artificial intelligence. As the founder of MetaLog Inc.,
              he has been at the forefront of developing intelligent systems that bridge the gap between AI and robotics.
              This book represents years of hands-on experience and research in humanoid robotics, from fundamental
              concepts to state-of-the-art vision-language-action systems.
            </p>
            <div className={styles.authorContact}>
              <h4>Get in Touch</h4>
              <div className={styles.contactLinks}>
                <a href="mailto:osqazi@gmail.com" className={styles.contactLink}>
                  <span className={styles.contactIcon}>✉️</span>
                  osqazi@gmail.com
                </a>
                <a href="https://wa.me/923353221003" target="_blank" rel="noopener noreferrer" className={styles.contactLink}>
                  <span className={styles.contactIcon}>📱</span>
                  +92 335 3221003
                </a>
                <a href="https://www.facebook.com/metalogai" target="_blank" rel="noopener noreferrer" className={styles.contactLink}>
                  <span className={styles.contactIcon}>📘</span>
                  MetaLog AI on Facebook
                </a>
              </div>
            </div>
          </div>
        </div>
      </div>
    </section>
  );
}

function CallToAction() {
  return (
    <section className={styles.ctaSection}>
      <div className="container">
        <h2 className={styles.ctaTitle}>Ready to Build Humanoid Robots?</h2>
        <p className={styles.ctaDescription}>
          Start your journey into the world of humanoid robotics with practical, hands-on guidance.
        </p>
        <div className={styles.buttons}>
          <Link
            className="button button--primary button--lg"
            to="/docs/intro">
            Get Started Now
          </Link>
        </div>
      </div>
    </section>
  );
}

export default function Home(): JSX.Element {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={`${siteConfig.title}`}
      description="A comprehensive guide to building intelligent humanoid robots using ROS 2, NVIDIA Isaac, and Vision-Language-Action systems.">
      <HomepageHeader />
      <main>
        <BookFeatures />
        <AuthorSection />
        <CallToAction />
      </main>
    </Layout>
  );
}
