import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import Translate, {translate} from '@docusaurus/Translate';
import styles from './index.module.css';

function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header className={clsx('hero hero--primary', styles.heroBanner)}>
      <div className="container">
        <h1 className={styles.heroTitle}>{siteConfig.title}</h1>
        <p className={styles.heroSubtitle}>{siteConfig.tagline}</p>
        <p className={styles.heroDescription}>
          <Translate id="homepage.hero.description">
            A comprehensive guide to building intelligent humanoid robots using modern robotics frameworks and AI systems.
          </Translate>
        </p>
        <div className={styles.buttons}>
          <Link
            className="button button--secondary button--lg"
            to="/docs/intro">
            <Translate id="homepage.hero.startReading">
              Start Reading
            </Translate>
          </Link>
          <Link
            className={clsx('button button--outline button--secondary button--lg', styles.buttonGithub)}
            to="https://github.com/osqazi/book_hackathon">
            <Translate id="homepage.hero.viewGithub">
              View on GitHub
            </Translate>
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
        <h2 className={styles.sectionTitle}>
          <Translate id="homepage.features.title">
            What You'll Learn
          </Translate>
        </h2>
        <div className={styles.featureGrid}>
          <div className={styles.featureCard}>
            <div className={styles.featureIcon}>🤖</div>
            <h3>
              <Translate id="homepage.features.ros2.title">
                ROS 2 Fundamentals
              </Translate>
            </h3>
            <p>
              <Translate id="homepage.features.ros2.description">
                Master Robot Operating System 2, the industry-standard framework for robotics development.
              </Translate>
            </p>
          </div>
          <div className={styles.featureCard}>
            <div className={styles.featureIcon}>🎮</div>
            <h3>
              <Translate id="homepage.features.simulation.title">
                Simulation & Testing
              </Translate>
            </h3>
            <p>
              <Translate id="homepage.features.simulation.description">
                Learn to simulate and test humanoid robots in virtual environments using Gazebo and Isaac Sim.
              </Translate>
            </p>
          </div>
          <div className={styles.featureCard}>
            <div className={styles.featureIcon}>⚡</div>
            <h3>
              <Translate id="homepage.features.isaac.title">
                NVIDIA Isaac Platform
              </Translate>
            </h3>
            <p>
              <Translate id="homepage.features.isaac.description">
                Leverage GPU-accelerated robotics simulation and AI training with NVIDIA's Isaac platform.
              </Translate>
            </p>
          </div>
          <div className={styles.featureCard}>
            <div className={styles.featureIcon}>👁️</div>
            <h3>
              <Translate id="homepage.features.vla.title">
                Vision-Language-Action
              </Translate>
            </h3>
            <p>
              <Translate id="homepage.features.vla.description">
                Integrate cutting-edge VLA models to enable robots to understand and act on natural language commands.
              </Translate>
            </p>
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
            <h2 className={styles.authorTitle}>
              <Translate id="homepage.author.title">
                About the Author
              </Translate>
            </h2>
            <h3 className={styles.authorName}>Owais Qazi</h3>
            <p className={styles.authorRole}>
              <Translate id="homepage.author.role">
                Founder & CEO, MetaLog Inc.
              </Translate>
            </p>
            <p className={styles.authorBio}>
              <Translate id="homepage.author.bio">
                Owais Qazi is a leading expert in robotics and artificial intelligence. As the founder of MetaLog Inc.,
                he has been at the forefront of developing intelligent systems that bridge the gap between AI and robotics.
                This book represents years of hands-on experience and research in humanoid robotics, from fundamental
                concepts to state-of-the-art vision-language-action systems.
              </Translate>
            </p>
            <div className={styles.authorContact}>
              <h4>
                <Translate id="homepage.author.contact.title">
                  Get in Touch
                </Translate>
              </h4>
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
                  <Translate id="homepage.author.contact.facebook">
                    MetaLog AI on Facebook
                  </Translate>
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
        <h2 className={styles.ctaTitle}>
          <Translate id="homepage.cta.title">
            Ready to Build Humanoid Robots?
          </Translate>
        </h2>
        <p className={styles.ctaDescription}>
          <Translate id="homepage.cta.description">
            Start your journey into the world of humanoid robotics with practical, hands-on guidance.
          </Translate>
        </p>
        <div className={styles.buttons}>
          <Link
            className="button button--primary button--lg"
            to="/docs/intro">
            <Translate id="homepage.cta.getStarted">
              Get Started Now
            </Translate>
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
      description={translate({
        id: 'homepage.meta.description',
        message: 'A comprehensive guide to building intelligent humanoid robots using ROS 2, NVIDIA Isaac, and Vision-Language-Action systems.',
      })}>
      <HomepageHeader />
      <main>
        <BookFeatures />
        <AuthorSection />
        <CallToAction />
      </main>
    </Layout>
  );
}
