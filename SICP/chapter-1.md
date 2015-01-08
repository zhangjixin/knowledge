#Chapter 1 -- Building Abstractions with Procedures
>> The acts of the mind, wherin it exerts its power over simple ideas, are chiefly these three: 1. Combining several simple ideas into one compound one, and thus all complex ideas are made. 2. The second is bringing two ideas, whether simple or complex, together, and setting them by one another so as to take a view of them at once, without uniting them into one, by which it gets all its ideas of relations. 3. The third is separating them from all other ideas that accompany them in their real existence: this is called abstraction, and thus all its general ideas are made.

We are about to study the idea of *computational process*. Computational processes are abstract beings that inhabit computers. As they evolve, processes manipulate other abstract things called *data*. The evolution of a process is directed by a pattern of rules called program. People create programs to direct processes.

Well-designed computational systems are designed in a modular manner, so that the parts can be constructed, replaced, and debugged separately.

###**_Programming in Lisp_**

Lisp was invented in the late 1950s as a formalism for reasoning about the use of certain kinds of logical expressions, called *recursion equations*, as a model for computation. The language was conceived by John McCarthy and is based on his paper `Recursive Functions of Symbolic Expressions and Their Computation by Machine` (McCarthy 1960).

Liso, whose name is an acronym for LISt Processing, was designed to provide symbol-manipulating capabilities for attacking programming problems such as the symbolic differentiation and integration of algebraic expressions. It included for this purpose new data objects known as *atoms* and *lists*, which most strikingly set it apart from all other languages of the period.

Lisp was not the product of a concerted design effort. Instead, it evolved informally in an experimental manner in response to user's needs and to pragmatic implementation considerations. Lisp's informal evolution has continued through the years, and the community of Lisp users has traditionally resisted attempts to promulgate any *official* definition of the language. This evolution, together with the flexibility and elegance of the initial conception, has enabled Lisp to continually adapt to encompass most modern ideas about program design. Thus, Lisp is by now a family of dialects, which, while sharing most of the original features, may differ from one another in significant ways. The dialect of Lisp used in this book is called **Scheme**.

Because of its experimental character and its emphasis on symbol manipulation, Lisp was at first very inefficient for numerical computations, at least in comparison with Fortran. Although Lisp has not yet overcome its old reputaion as hopelessly inefficient, Lisp is now used in many applications where efficiency is not the central concern.

The most significant feature is the fact that Lisp descriptions of processed, called *procedures*, can themselves be represented and manipulated as Lisp data.


