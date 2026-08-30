# GalvaKnights 2026 REBUILT

GalvaKnights 2026 competition code, in [Kotlin](https://kotlinlang.org/).

# Why Kotlin?
While Kotlin isn't officially supported by FRC, it is quickly growing popularity with many other teams, 
including [Team 2471](https://github.com/TeamMeanMachine), [Team 2521](https://www.sert2521.org/), and [Team 2537](https://team2537.com/).

Kotlin is a modern programming language that has many features that make it easier to write and maintain. There are many benefits over Java, such as:
- **Null Safety**: Kotlin has built-in null safety features that help prevent null pointer exceptions, which are a common source of bugs in Java.
- **Type Inference**: Kotlin can type infer variables, allowing cleaner code and less boilerplate.
- **Coroutines**: Kotlin has built-in support for coroutines, which makes it easier to write asynchronous code. This is especially useful for IO-bound tasks, such as vision requests.

Kotlin is also fully interoperable with Java, which means you can use all existing Java libraries and frameworks.
This means that you can use existing Java classes in your Kotlin code, and vice versa.

# Getting Started

While you can work on this project in the *2025 WPILib VS Code* editor, we recommend using the [IntelliJ IDEA](https://www.jetbrains.com/idea/) IDE because of its native support for Kotlin. If you do not have an IntelliJ subscription, the community edition has all the features needed.

## IDE Setup

You will need to install certain plugins to properly work within this project:

**FRC** *(Optional)* - Provides class and object templates and a RioLog console
**detekt** *(Required)* - Code linter that provides static analysis for conventions
**Ktlint** *(Required)* - Code linter that enforces and fixes styling conventions
**Spotless Gradle** *(Required)* - Code linter that enforces and fixes broader styling conventions
## Gradle Configuration

In the IDE settings, set the **Gradle JVM** to **Java 25**. If not prompted, make sure you run **Sync All Gradle Projects** to make sure *Code Completion* works properly. You will need to run this whenever you update your dependencies.

## Grabbing dependencies and vendordeps

To install all dependencies, run the **Build Robot** Gradle configuration.

## Gradle Tasks

To run gradle tasks, you can use the **Gradle** tool window in IntelliJ IDEA. This will allow you to run tasks such as `Deploy Robot`, `Build Robot`, and `Clean`.
