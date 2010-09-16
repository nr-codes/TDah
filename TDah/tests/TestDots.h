/** @file TestDots.h
* @brief the test suite for the Dot and Dots classes
*
* This file is the input to the CxxTest script that is
* used to create the TestDots.cpp file
*
* @note the TestDots.cpp file in the project is a place
* holder for the auto-generatated .cpp file from CxxTest
* using its python script.  This is done in V2K8 by running
* a pre-build command in the build event properties setting.
* The command (all on one line) executed is:
 @verbatim
  c:\cygwin\bin\python2.5.exe ../../tests/cxxtest/cxxtestgen.py 
  -o ../../tests/TestDots.cpp --gui=Win32Gui 
  --runner=ParenPrinter ../../tests/TestDots.h
 @endverbatim
*
* where the relative directory is assumed to be from where the
* Visual Studio project "Dots.vcproj" is located, which is currently
* ./TDah/build/vs2k8.
*/

#include <cxxtest/TestSuite.h>
#include "Dots.h"

/**
* Returns a count of the unique tag numbers created by makeDots().
* After the function returns all dots are in the active set.
*
* @param[in] d the set of Dots to check for uniqueness amoung its tags.
* @return the number of unique tags
* @note the caller is expected to call makeDots() before calling this function
*/

int count_unique_tags(Dots& d)
{
	int n = 0;
	int* tags;
	ActiveDots& a = d.activeDots();

	// we can get indirect access to each dot via the active set
	d.makeAllDotsActive();
	tags = new int [a.size()];

	// copy tags for unique tag number test
	for(size_t i = 0; i < a.size(); i++) {
		tags[i] = a.at(i).tag();
	}

	// sort tags so that equal tag numbers are in consecutive order
	std::sort( tags, tags + a.size());

	// order the unique tag values in a subvector within the original vector
	n = std::unique( tags, tags + a.size() ) - tags;

	delete [] tags;
	return n;
}

class TestDotsSuite : public CxxTest::TestSuite 
{
public:
	void testMakeDots1( void )
	{
		int n;
		Dots d;

		// create n dots and make sure active_set is always empty
		n = 10;
		d.makeDots( n );
		TS_ASSERT( d.activeDots().empty() );

		n = 3;
		d.makeAllDotsActive();
		d.makeDots( n );
		TS_ASSERT( d.activeDots().empty() );
	}

	void testMakeDots2( void )
	{
		int n;
		Dots d;
		d.makeDots( 1000 );
		
		// check to make sure the number of unique tags is 
		// also the size of the number of dots
		n = count_unique_tags( d );
		TS_ASSERT_EQUALS( n, d.activeDots().size() );
	}

	void testMakeDotActive1( void )
	{
		int n = 44;
		Dots d;
		ActiveDots& a = d.activeDots();

		// create dots
		d.makeDots( n );

		// add 1 dot to active set and make sure it has the right tag info
		d.makeDotActive( n / 5 );
		TS_ASSERT( !a.empty() );
		TS_ASSERT_EQUALS( a.size(), 1 );
		TS_ASSERT_EQUALS( a[0].tag(), n / 5 );
	}

	void testMakeDotActive2( void )
	{
		int i = 79;
		Dots d;
		ActiveDots& a = d.activeDots();

		// create dots
		d.makeDots( 810 );

		// add the same dot and make sure a duplicate is not in the active set
		int j = 7;
		while(j--) d.makeDotActive( i );
		TS_ASSERT_EQUALS( a.size(), 1 );
		TS_ASSERT_EQUALS( a[0].tag(), i );
	}

	void testMakeDotActive3( void )
	{
		int i = 4;
		Dots d;
		ActiveDots& a = d.activeDots();

		// create dots
		d.makeDots( 10 );

		// add 1 dot to active set and make sure it has the right tag info
		d.makeDotActive( i );

		// make sure tags that are out-of-bounds crashes the function
		TS_ASSERT_THROWS_ANYTHING( d.makeDotActive( -i ) ); 
		TS_ASSERT_THROWS_ANYTHING( d.makeDotActive( a.size() + 37 ) );

		// make sure errors didn't change anything
		TS_ASSERT_EQUALS( a.size(), 1 );
		TS_ASSERT_EQUALS( a[0].tag(), i );
	}

	void testMakeAllDotsActive1( void )
	{
		int n = 500;
		Dots d;
		d.makeDots( n );

		// add all dots to the list
		d.makeAllDotsActive();
		TS_ASSERT_EQUALS( d.activeDots().size(), n );

		// clear the dots, add one dot, and make sure its not added twice
		d.clearActiveDots();
		d.makeDotActive( n / 2);
		d.makeAllDotsActive();

		n = count_unique_tags( d );
		TS_ASSERT_EQUALS( d.activeDots().size(), n );
	}

	void testClearActiveDots( void )
	{
		Dots d;
		Dot t;

		// create dots
		d.makeDots( 127 );

		// test that clearing active dots always makes the active set empty
		d.makeAllDotsActive();
		TS_ASSERT( !d.activeDots().empty() );

		t = d.activeDots().front();
		d.clearActiveDots();
		TS_ASSERT( d.activeDots().empty() );

		d.makeDotActive( t.tag() );
		TS_ASSERT( !d.activeDots().empty() );

		d.clearActiveDots();
		TS_ASSERT( d.activeDots().empty() );
	}
};
